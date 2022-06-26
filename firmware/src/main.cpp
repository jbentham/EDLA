// ESP32 Web-based remote logic analyser
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
//
// v0.76 JPB 26/6/22 Tidied up for release

#define SW_VERSION   "0.76"

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp32_la.h"
#include "esp32_web.h"

#define PIN_DEBUG_RX    26
#define PIN_DEBUG_TX    27
#define DEBUG_BAUD      115200

uint32_t ledticks;
bool ledstate;
char version[] = "ESP32_LA v" SW_VERSION;

bool mstimeout(uint32_t *tickp, uint32_t tout);

void setup() {
    Serial.begin(DEBUG_BAUD);
    debug_init();
    DEBUG.println(version);
    spi_bus_init();
    net_start();
    mstimeout(&ledticks, 0);
    while (!net_ready()) {
        net_check();
        if (mstimeout(&ledticks, 100))
            set_led(ledstate = !ledstate);
    }
    net_check();
    set_led(ledstate=0);
    dac_init();
    cap_init();
    ram_data_sim(0, RAM_NSAMP);
    set_threshold(THRESH_DEFAULT);
    DEBUG.printf("Attenuator value %u:1\r\n", THRESH_SCALE);
}

void loop() {
    STATE_VALS state = get_state();
    if (!net_check()) {
        if (mstimeout(&ledticks, 100))
            set_led(ledstate = !ledstate);
    }
    else if ((state==STATE_IDLE || state==STATE_READY) && mstimeout(&ledticks, ledstate?50:2000))
        set_led(ledstate = !ledstate);
    web_poll();
}

void debug_init(void) {
#if EXT_SERIAL_CONSOLE
    DEBUG.begin(DEBUG_BAUD, SERIAL_8N1, PIN_DEBUG_RX, PIN_DEBUG_TX);
#endif
}

void debug_printf(const char *format, ...) {
#ifdef DEBUG
    char buffer[256];
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(buffer, sizeof(buffer) - 1, format, argptr);
    DEBUG.write(buffer);
    va_end(argptr);
#endif
}

// If msec timer has expired, reload it and return true
bool mstimeout(uint32_t *tickp, uint32_t tout) {
    uint32_t t = millis(), diff = t-*tickp;
    if (!tout || diff>=tout)
        *tickp = t;
    return(diff>=tout);
}

// EOF
