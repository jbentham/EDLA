// ESP32 logic analyser interface definitions
// See https://iosoft.blog for details
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

// External memory
// 23LC1024 is 1 Mbit, and there are 2 samples per addressable byte
#define RAM_MAX_ADDR    0x20000
#define RAM_NSAMP       (RAM_MAX_ADDR * 2)

// Logic analyser
#define NUM_RAMS        4       // Number of RAM chips
#define NCHANS          (NUM_RAMS * 4)
#define TXBUFF_NSAMP    702
#define TXBUFF_LEN      (TXBUFF_NSAMP * 2)
#define MAX_SAMP_RATE   20000000
//#define THRESH_SCALE    7       // Divisor to convert threshold mV to DAC mV
#define THRESH_SCALE    101
#define THRESH_DEFAULT  10      // Default threshold in V

// Select serial console on USB or external serial
#define EXT_SERIAL_CONSOLE  0
#if EXT_SERIAL_CONSOLE
#define DEBUG       Serial2     // Debug on external serial link
#else
#define DEBUG       Serial      // Debug on USB serial link
#endif

// Parameters for sample capture
typedef struct {
    uint32_t xsamp, xrate;
} CAP_PARAMS;

void debug_init(void);
void debug_printf(const char *format, ...);

void ram_data_sim(int oset, int count);
bool ram_test(void);
void ram_write_bytes(int oset, byte *data, int len);
void ram_read_bytes(int oset, byte *data, int len);
void ram_write_qspi_words(int oset, uint16_t *data, int count);
void ram_read_qspi_words(int oset, uint16_t *data, int count);
void ram_read_start(int oset);
void ram_send_cmd(byte *cmd, int len);
void ram_send_bytes(byte *data, int len);
void ram_recv_bytes(byte *data, int len);
void ram_write_start(int oset);
void ram_write_end(void);
void ram_read_end(void);
void pins_config(uint64_t ins, uint64_t outs);
void pins_reconfig(uint64_t ins, uint64_t outs);
inline void gpio_out_din(uint32_t val);
inline void gpio_out_bus(uint32_t val);
void spi_bus_init(void);
void qspi_bus_init(void);
inline uint32_t word_busval(uint16_t w);
inline uint16_t bus_wordval(uint32_t w);
int busbit_pin(int bitnum);
void bus_send_spi_cmd(byte *cmd, int len);
void bus_send_spi_bytes(byte *txd, int len);
void bus_recv_spi_bytes(byte *data, int len);
void bus_send_qspi_cmd(byte *cmd, int len);
void bus_send_qspi_bytes(byte *txd, int len);
void bus_recv_qspi_bytes(byte *rxd, int len);
void bus_send_qspi_sim(int count);
void bus_send_qspi_words(uint16_t *txd, int count);
void bus_recv_qspi_words(uint16_t *rxd, int count);
void qspi_begin(void);
void qspi_input(void);
void qspi_output(void);
void qspi_end(void);
void pcnt_init(int pin);
void pcnt_reset(void);
uint32_t pcnt_val32(void);
void pwm_init(int pin, int freq);
void pwm_start(void);
void pwm_stop(void);

void cap_init(void);
void cap_start(CAP_PARAMS *cp);
void cap_end(void);
void cap_read_start(int addr);
void cap_read_block(uint16_t *buff, int nsamp);
void cap_read_end(void);
uint32_t samp_wrap(uint32_t n);

void dac_init(void);
void dac_out(int mv);
void set_threshold(int v);
void set_led(bool on);
void dump_bytes(byte *data, int len);
void dump_words(uint16_t *data, int len);
uint16_t prng16(uint16_t x);

// EOF
