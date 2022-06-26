// ESP32 logic analyser hardware interface
// See https://iosoft.blog/edla for details
//
// Copyright (c) 2022 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include "driver/pcnt.h"
#include <spi.h>
#include "esp32_la.h"

// If non-zero, fake data for testing
#define FAKE_DATA       0

// Pin definitions
#define PIN_DAC_CS      2
#define PIN_RAM_CS      32
#define PIN_SCK         33
#define PIN_SCK_IN      34
#define LOWEST_BUS_PIN  4
#define HIGHEST_BUS_PIN 27

// PWM (data acquisition clock) definitions
#define PWM_TEST_FREQ   10000000
#define PWM_CHAN        0

// Pulse counter (RAM address tracker) definitions
#define PCNT_UNIT       PCNT_UNIT_0
#define PCNT_CHAN       PCNT_CHANNEL_0
#define PCNT_TRIG_UNIT  PCNT_UNIT_1

// Data pin numbers for one RAM chip
#define RAM_SPI_DIN     0
#define RAM_SPI_DOUT    1
#define RAM_SPI_HOLD    3
// Corresponding SPI bit masks
#define MSK_SPI_DOUT    (1 << RAM_SPI_DIN)
#define MSK_SPI_DIN     (1 << RAM_SPI_DOUT)
#define MSK_SPI_HOLD    (1 << RAM_SPI_HOLD)
#define MSK_SPI_IN      (MSK_SPI_DIN)
#define MSK_SPI_OUTS    (MSK_SPI_DOUT | MSK_SPI_HOLD)
#define MSK_SQI_IOS     0xf
#if NUM_RAMS == 3
    #define ALL_RAM_WORD(b) ((b) | (b)<<4 | (b)<<8)
#else
    #define ALL_RAM_WORD(b) ((b) | (b)<<4 | (b)<<8 | (b)<<12)
#endif
#define RAM_TEST_OSET   0x1bcde

// I/O macros
#define DAC_SELECT      GPIO.out_w1tc = 1<<PIN_DAC_CS
#define DAC_DESELECT    GPIO.out_w1ts = 1<<PIN_DAC_CS
#define RAM_SELECT      GPIO.out1_w1tc.val = 1<<(PIN_RAM_CS-32)
#define RAM_DESELECT    GPIO.out1_w1ts.val = 1<<(PIN_RAM_CS-32)
#define SET_SCK         GPIO.out1_w1ts.val = 1<<(PIN_SCK-32)
#define CLR_SCK         GPIO.out1_w1tc.val = 1<<(PIN_SCK-32)
#define NO_PULLS        GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE
#define PULL_DOWN       GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_ENABLE, GPIO_INTR_DISABLE

// Data bus pin definitions
typedef struct {
    uint32_t z1:4, d0_1:2, z2:6, d2_9:8, z3:1, d10_12:3, z4:1, d13_15:3;
} BUSPINS;
typedef union {
    uint32_t val;
    BUSPINS pins;
} BUSPINVAL;
// Matching elements in 16-bit word
typedef struct {
    uint32_t d0_1:2, d2_9:8, d10_12:3, d13_15:3;
} BUSWORD;
typedef union {
    uint16_t val;
    BUSWORD bits;
} BUSWORDVAL;

char hexchar[] = "0123456789ABCDEF";
uint64_t input_pins, output_pins;
bool qspi_mode;
volatile uint16_t pcnt_hi_word;

uint32_t spi_in_pins   = word_busval(ALL_RAM_WORD(MSK_SPI_IN));
uint32_t spi_out_pins  = word_busval(ALL_RAM_WORD(MSK_SPI_OUTS));
uint32_t spi_dout_pins = word_busval(ALL_RAM_WORD(MSK_SPI_DOUT));
uint32_t spi_hold_pins = word_busval(ALL_RAM_WORD(MSK_SPI_HOLD));
uint32_t sqi_io_pins   = word_busval(ALL_RAM_WORD(MSK_SQI_IOS));

// Load simulated data into RAM
void ram_data_sim(int oset, int count) {
    qspi_begin();
    ram_write_start(oset);
    bus_send_qspi_sim(count);
    ram_write_end();
    qspi_end();
}

// Do a RAM test, return 0 if failed
bool ram_test(void) {
    uint8_t obytes[12] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
        0x11, 0x22, 0x44, 0x88 };
    uint8_t ibytes[12];
    uint16_t iwords[6];
#if NUM_RAMS == 3
    uint16_t owords[6] = { 0x0234, 0x0345, 0x0456, 0x0678, 0x0789, 0x089a };
    uint16_t idata[6] = { 0x0000, 0x0421, 0x0210, 0x0008, 0x0184, 0x0100 };
#else
    uint16_t owords[6] = { 0x1234, 0x2345, 0x3456, 0x5678, 0x6789, 0x789a };
    uint16_t idata[6] = { 0x0000, 0x8421, 0x8421, 0x0000, 0x8421, 0x8421 };
#endif
    int fail;
    spi_bus_init();
    ram_write_bytes(RAM_TEST_OSET, obytes, sizeof(obytes));
    ram_read_bytes(RAM_TEST_OSET, ibytes, sizeof(ibytes));
    if ((fail = memcmp(obytes, ibytes, sizeof(obytes)))) {
        debug_printf("RAM SPI readback error\r\n");
        dump_bytes(obytes, sizeof(obytes));
        dump_bytes(ibytes, sizeof(ibytes));
    }
    qspi_begin();
    ram_write_qspi_words(RAM_TEST_OSET + 0x20, owords, sizeof(owords) / 2);
    ram_read_qspi_words(RAM_TEST_OSET + 0x20, iwords, sizeof(iwords) / 2);
    if (!fail && (fail = memcmp(owords, iwords, sizeof(owords)))) {
        debug_printf("RAM QSPI readback error\r\n");
        dump_words(owords, sizeof(owords) / 2);
        dump_words(iwords, sizeof(iwords) / 2);
    }
    ram_read_qspi_words(RAM_TEST_OSET, iwords, sizeof(iwords) / 2);
    qspi_end();
    if (!fail && (fail = memcmp(iwords, idata, sizeof(iwords)))) {
        debug_printf("RAM SPI to QSPI error\r\n");
        dump_words(idata, sizeof(idata) / 2);
        dump_words(iwords, sizeof(iwords) / 2);
    }
    return (!fail);
}

// Write block of bytes to RAM
void ram_write_bytes(int oset, byte *data, int len) {
    ram_write_start(oset);
    ram_send_bytes(data, len);
    ram_write_end();
}

// Read block of bytes from RAM
void ram_read_bytes(int oset, byte *data, int len) {
    ram_read_start(oset);
    ram_recv_bytes(data, len);
    ram_read_end();
}

// Write block of 16-bit words to RAM in QSPI mode
void ram_write_qspi_words(int oset, uint16_t *data, int count) {
    ram_write_start(oset);
    bus_send_qspi_words(data, count);
    ram_write_end();
}

// Read block of 16-bit words from RAM in QSPI mode
void ram_read_qspi_words(int oset, uint16_t *data, int count) {
    ram_read_start(oset);
    bus_recv_qspi_words(data, count);
    ram_read_end();
}

// Start SPI or QSPI memory write, given offset (number of samples)
void ram_write_start(int n) {
    byte cmd[4] = { 2, (byte)(n >> 17), (byte)(n >> 9), (byte)(n >> 1) };
    RAM_SELECT;
    ram_send_cmd(cmd, sizeof(cmd));
}

// Start SPI or QSPI memory read, given offset (number of samples)
void ram_read_start(int n) {
    byte cmd[4] = { 3, (byte)(n >> 17), (byte)(n >> 9), (byte)(n >> 1) };
    byte padding[NUM_RAMS];
    RAM_SELECT;
    ram_send_cmd(cmd, sizeof(cmd));
    if (qspi_mode) {
        qspi_input();
        bus_recv_qspi_bytes(padding, sizeof(padding));
    }
}

// Send command to all RAMs using SPI or QSPI
void ram_send_cmd(byte *cmd, int len) {
    if (qspi_mode)
        bus_send_qspi_cmd(cmd, len);
    else
        bus_send_spi_cmd(cmd, len);
}

// Write bytes to memory using SPI or QSPI
// Length value is total length in bytes for all RAMs
void ram_send_bytes(byte *data, int len) {
    if (qspi_mode)
        bus_send_qspi_bytes(data, len);
    else
        bus_send_spi_bytes(data, len);
}

// Read bytes from memory using SPI or QSPI
// Length value is total length in bytes for all RAMs
void ram_recv_bytes(byte *data, int len) {
    if (qspi_mode)
        bus_recv_qspi_bytes(data, len);
    else
        bus_recv_spi_bytes(data, len);
}

// End write to RAM
void ram_write_end(void) {
    RAM_DESELECT;
}

// End read from RAM
void ram_read_end(void) {
    if (qspi_mode)
        qspi_output();
    RAM_DESELECT;
}

// Set input & output pins, given bitmaps
// Takes around 70 usec
void pins_config(uint64_t ins, uint64_t outs) {
    gpio_config_t icfg = { ins, GPIO_MODE_INPUT, NO_PULLS };
    gpio_config_t ocfg = { outs, GPIO_MODE_OUTPUT, NO_PULLS };
    if (ins)
        gpio_config(&icfg);
    if (outs)
        gpio_config(&ocfg);
    input_pins = ins;
    output_pins = outs;
    gpio_set_direction((gpio_num_t)PIN_RAM_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_SCK, GPIO_MODE_OUTPUT);
}

// Change bus pins between input and output
// Takes around 1 usec per pin change
void pins_reconfig(uint64_t ins, uint64_t outs) {
    uint64_t ips = ins & ~input_pins, ops = outs & ~output_pins;
    for (int i = LOWEST_BUS_PIN; i <= HIGHEST_BUS_PIN && (ips || ops); i++) {
        uint64_t msk = 1 << i;
        if (ips & msk)
            gpio_set_direction((gpio_num_t)i, GPIO_MODE_INPUT);
        if (ops & msk)
            gpio_set_direction((gpio_num_t)i, GPIO_MODE_OUTPUT);
    }
    input_pins = ins;
    output_pins = outs;
}

// Send a 32-bit value to all RAM SPI inputs
inline void gpio_out_spi(uint32_t val) {
    GPIO.out_w1ts = val  & spi_dout_pins;
    GPIO.out_w1tc = ~val & spi_dout_pins;
}

// Send a 32-bit value to all RAM SQI inputs
inline void gpio_out_bus(uint32_t val) {
    GPIO.out_w1ts = val  & sqi_io_pins;
    GPIO.out_w1tc = ~val & sqi_io_pins;
}

// Set I/O bus configuration for SPI
void spi_bus_init(void) {
    CLR_SCK;
    pins_config(spi_in_pins, spi_out_pins);
    qspi_mode = 0;
}

// Set I/O bus configuration for QSPI
void qspi_bus_init(void) {
    RAM_DESELECT;
    CLR_SCK;
    pins_reconfig(0, sqi_io_pins);
}

// Return 32-bit bus I/O value, given 16-bit word
inline uint32_t word_busval(uint16_t val) {
    BUSWORDVAL w = { .val = val };
    BUSPINVAL  p = { .pins = { 0, w.bits.d0_1,   0, w.bits.d2_9,
                               0, w.bits.d10_12, 0, w.bits.d13_15 } };
    return (p.val);
}

// Return 16-bit word, given 32-bit bus I/O value
inline uint16_t bus_wordval(uint32_t val) {
    BUSPINVAL  p = { .val = val };
    BUSWORDVAL w = { .bits = { p.pins.d0_1, p.pins.d2_9, 
                               p.pins.d10_12, p.pins.d13_15 } };
#if NUM_RAMS == 3
    val &= 0xfff;
#endif
    return (w.val);
}

// Return I/O pin number, given bus bit number
int busbit_pin(int bitnum) {
    uint32_t val = word_busval(1 << bitnum);
    int n=0;
    while (val) {
        n++;
        val >>= 1;
    }
    return(n-1);
}

// Send byte command to all RAMs using SPI
// Toggles SPI clock at around 7 MHz
void bus_send_spi_cmd(byte *cmd, int len) {
    GPIO.out_w1ts = spi_hold_pins;
    while (len--) {
        byte b = *cmd++;
        for (int n = 0; n < 8; n++) {
            if (b & 0x80) GPIO.out_w1ts = spi_dout_pins;
            else GPIO.out_w1tc = spi_dout_pins;
            SET_SCK;
            b <<= 1;
            CLR_SCK;
        }
    }
}

// Send bytes to RAM chips using SPI
// Length value is total length in bytes for all RAMs
void bus_send_spi_bytes(byte *txd, int len) {
    uint16_t w;
    len /= NUM_RAMS;
    while (len--) {
#if NUM_RAMS == 3
        byte b0 = txd[0], b1 = txd[1], b2 = txd[2];
#else
        byte b0 = txd[0], b1 = txd[1], b2 = txd[2], b3 = txd[3];
#endif
        for (int n = 0; n < 8; n++) {
#if NUM_RAMS == 3
            w = (b0 >> 7 & 1) | (b1 >> 7 & 1) << 4 | (b2 >> 7 & 1) << 8;
#else
            w = (b0 >> 7 & 1) | (b1 >> 7 & 1) << 4 | (b2 >> 7 & 1) << 8 | (b3 >> 7 & 1) << 12;
#endif
            gpio_out_spi(word_busval(w));
            SET_SCK;
            b0 <<= 1;
            b1 <<= 1;
            b2 <<= 1;
#if NUM_RAMS > 3
            b3 <<= 1;
#endif
            CLR_SCK;
        }
        txd += NUM_RAMS;
    }
}

// Receive bytes from RAM chips using SPI
// Length value is total length in bytes for all RAMs
void bus_recv_spi_bytes(byte *rxd, int len) {
    uint16_t w;
    len /= NUM_RAMS;
    while (len--) {
#if NUM_RAMS == 3
        rxd[0] = rxd[1] = rxd[2] = 0;
#else
        rxd[0] = rxd[1] = rxd[2] = rxd[3] = 0;
#endif
        for (int n = 0; n < 8; n++) {
            SET_SCK;
            w = bus_wordval(GPIO.in & spi_in_pins) >> RAM_SPI_DOUT;
            CLR_SCK;
            rxd[0] = rxd[0] << 1 | (w & 1);
            rxd[1] = rxd[1] << 1 | (w >> 4 & 1);
            rxd[2] = rxd[2] << 1 | (w >> 8 & 1);
#if NUM_RAMS > 3
            rxd[3] = rxd[3] << 1 | (w >> 12 & 1);
#endif
        }
        rxd += NUM_RAMS;
    }
}

// Send a single command to all RAMs using QSPI
void bus_send_qspi_cmd(byte *cmd, int len) {
    while (len--) {
        uint32_t b1 = *cmd >> 4, b2 = *cmd & 15, val = word_busval(ALL_RAM_WORD(b1));
        gpio_out_bus(val);
        SET_SCK;
        val = word_busval(ALL_RAM_WORD(b2));
        CLR_SCK;
        gpio_out_bus(val);
        SET_SCK;
        cmd++;
        CLR_SCK;
    }
}

// Send bytes to RAM chips using QSPI
// Length value is total length in bytes for all RAMs
void bus_send_qspi_bytes(byte *txd, int len) {
    uint16_t w;
    len /= NUM_RAMS;
    while (len--) {
#if NUM_RAMS == 3
        w = (txd[0] >> 4 & 0xf) | (txd[1] & 0xf0) | (txd[2] >> 4 & 0xf) << 8;
#else
        w = (txd[0] >> 4 & 0xf) | (txd[1] & 0xf0) | (txd[2] >> 4 & 0xf) << 8 | (txd[3] >> 4 & 0xf) << 12;
#endif
        gpio_out_bus(word_busval(w));
        SET_SCK;
#if NUM_RAMS == 3
        w = (txd[0] & 0xf) | (txd[1] & 0xf) << 4 | (txd[2] & 0xf) << 8;
#else
        w = (txd[0] & 0xf) | (txd[1] & 0xf) << 4 | (txd[2] & 0xf) << 8 | (txd[3] & 0xf) << 12;
#endif
        CLR_SCK;
        gpio_out_bus(word_busval(w));
        SET_SCK;
        txd += NUM_RAMS;
        CLR_SCK;
    }
}

// Receive bytes from RAM chips using QSPI
// Length value is total length in bytes for all RAMs
void bus_recv_qspi_bytes(byte *rxd, int len) {
    uint16_t w;
    len /= NUM_RAMS;
    while (len--) {
        SET_SCK;
        w = bus_wordval(GPIO.in & sqi_io_pins);
        CLR_SCK;
        rxd[0] = (w & 0xf) << 4;
        rxd[1] = (w & 0xf0);
        rxd[2] = (w & 0xf00) >> 4;
#if NUM_RAMS > 3
        rxd[3] = (w & 0xf000) >> 8;
#endif
        SET_SCK;
        w = bus_wordval(GPIO.in & sqi_io_pins);
        CLR_SCK;
        rxd[0] = rxd[0] | (w & 0xf);
        rxd[1] = rxd[1] | (w & 0xf0) >> 4;
        rxd[2] = rxd[2] | (w & 0xf00) >> 8;
#if NUM_RAMS > 3
        rxd[3] = rxd[3] | (w & 0xf000) >> 12;
#endif
        rxd += NUM_RAMS;
    }
}

// Send simulated data words to RAM chips using QSPI
void bus_send_qspi_sim(int count) {
    static int n=0;
    int i = 0;
    while (i < count) {
        gpio_out_bus(word_busval(i+n));
        SET_SCK;
        i++;
        CLR_SCK;
    }
    n += 100;
}

// Send words to RAM chips using QSPI
void bus_send_qspi_words(uint16_t *txd, int count) {
    while (count) {
        gpio_out_bus(word_busval(*txd++));
        SET_SCK;
        count--;
        CLR_SCK;
    }
}

// Receive words from RAM chips using QSPI
void bus_recv_qspi_words(uint16_t *rxd, int count) {
    while (count--) {
        SET_SCK;
        *rxd = bus_wordval(GPIO.in & sqi_io_pins);
        CLR_SCK;
#if FAKE_DATA == 1
        *rxd = 0xffff;
#endif
        rxd++;
    }
}

// Start QSPI mode
void qspi_begin(void) {
    RAM_DESELECT;
    RAM_DESELECT;
    RAM_SELECT;
    bus_send_spi_cmd((byte *)"\x38", 1);
    RAM_DESELECT;
    qspi_bus_init();
    qspi_mode = 1;
}

// Switch QSPI bus to input data from RAMs
// (takes around 70 usec)
void qspi_input(void) {
    pins_reconfig(sqi_io_pins, 0);
}

// Set QSPI bus to output data
void qspi_output(void) {
    pins_reconfig(0, sqi_io_pins);
}

// End QSPI mode
void qspi_end(void) {
    RAM_SELECT;
    bus_send_qspi_cmd((byte *)"\xff\xff", 2);
    RAM_DESELECT;
    spi_bus_init();
    qspi_mode = 0;
}

// Handler for PCNT interrupt
void IRAM_ATTR pcnt_handler(void *x) {
    uint32_t intr_status = PCNT.int_st.val;
    if (intr_status) {
        pcnt_hi_word++;
        PCNT.int_clr.val = intr_status;
    }
}

// Initialise PWM pulse counter
void pcnt_init(int pin) {
    pcnt_intr_disable(PCNT_UNIT);
    pcnt_config_t pcfg = { pin, PCNT_PIN_NOT_USED, PCNT_MODE_KEEP, PCNT_MODE_KEEP,
        PCNT_COUNT_INC, PCNT_COUNT_DIS, 0, 0, PCNT_UNIT, PCNT_CHAN };
    pcnt_unit_config(&pcfg);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, 0);
    pcnt_isr_register(pcnt_handler, 0, 0, 0);
    pcnt_intr_enable(PCNT_UNIT);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
    pcnt_hi_word = 0;
}

// Reset sample counter
void pcnt_reset(void) {
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_hi_word = 0;
    pcnt_counter_resume(PCNT_UNIT);
}

// Return sample counter value (mem addr * 2), extended to 32 bits
uint32_t pcnt_val32(void) {
    uint16_t hi = pcnt_hi_word, lo = PCNT.cnt_unit[PCNT_UNIT].cnt_val;
    if (hi != pcnt_hi_word)
        lo = PCNT.cnt_unit[PCNT_UNIT].cnt_val;
    return(((uint32_t)hi<<16) | lo);
}

// Initialise PWM output
void pwm_init(int pin, int freq) {
    ledcSetup(PWM_CHAN, freq, 1);
    ledcAttachPin(pin, PWM_CHAN);
}

// Start PWM output
void pwm_start(void) {
    ledcWrite(PWM_CHAN, 1);
}
// Stop PWM output
void pwm_stop(void) {
    ledcWrite(PWM_CHAN, 0);
}

// Set up capture
void cap_init(void) {
    pinMode(12, INPUT); // Set boot mode pin to I/O (D2)
    pcnt_init(PIN_SCK_IN);
}

// Start capture
void cap_start(CAP_PARAMS *cp) {
    qspi_begin();
    ram_write_start(0);
    qspi_input();
    pwm_init(PIN_SCK, cp->xrate);
    pcnt_reset();
#if FAKE_DATA == 2
    RAM_DESELECT;
#endif    
    pwm_start();
}

// Stop capturing data
void cap_end(void) {
    pwm_stop();
    qspi_output();
    qspi_end();
}

// Start reading captured data, given offset (number of samples)
void cap_read_start(int n) {
    spi_bus_init();
    qspi_begin();
    ram_read_start(n);
}

// Read block of captured data
void cap_read_block(uint16_t *buff, int nsamp) {
    bus_recv_qspi_words(buff, nsamp);
}

// End reading captured data
void cap_read_end(void) {
    ram_read_end();
}

// Wrap sample count around RAM boundary
uint32_t samp_wrap(uint32_t n) {
    return(n & (RAM_NSAMP-1));
}

// Initialise DAC hardware
void dac_init(void) {
    pinMode(PIN_DAC_CS, OUTPUT);
    digitalWrite(PIN_DAC_CS, 1);
}

// Output voltage from DAC; Vout = Vref * n / 4096
void dac_out(int mv) {
    uint16_t w = 0x7000 + ((mv * 4096) / 3300);
    byte cmd[2] = { (byte)(w >> 8), (byte)(w & 0xff) };
    RAM_DESELECT;
    DAC_SELECT;
    bus_send_spi_cmd(cmd, 2);
    DAC_DESELECT;
}

// Set threshold, given voltage
void set_threshold(int v) {
    dac_out(v * 1000 / THRESH_SCALE);
}

// Set or clear the LED (also selects RAM chips)
void set_led(bool on) {
    if (on)
        RAM_SELECT;
    else
        RAM_DESELECT;
}

// Dump byte values in hex
void dump_bytes(byte *data, int len) {
    while (len--) debug_printf("%02X ", *data++);
    debug_printf("\r\n");
}

// Dump 16-bit hex values
void dump_words(uint16_t *data, int len) {
    while (len--) debug_printf("%04X ", *data++);
    debug_printf("\r\n");
}

// 16-bit pseudo-random number generator
uint16_t prng16(uint16_t x) {
    x ^= x << 7;
    x ^= x >> 9;
    return (x ^= x << 8);
}

// EOF
