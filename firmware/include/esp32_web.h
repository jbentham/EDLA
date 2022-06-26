// Definitions for ESP32 Web interface
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

typedef enum {ARG_STATE, ARG_NSAMP, ARG_XSAMP, ARG_XRATE,
    ARG_THRESH, ARG_TRIGCHAN, ARG_TRIGMODE, ARG_TRIGPOS} ARG_VARS;

typedef enum {TRIG_NONE, TRIG_RISING, TRIG_FALLING} TRIG_MODES;

#define NUM_STATES  6
typedef enum {STATE_IDLE, STATE_READY, STATE_PRELOAD,
    STATE_PRETRIG, STATE_POSTTRIG, STATE_UPLOAD} STATE_VALS;

#define NUM_CMDS    3
typedef enum {CMD_STOP, CMD_SINGLE, CMD_MULTI} CMD_VALS;

void net_start(void);
bool net_check(void);
bool net_ready(void);
void web_poll(void);
bool web_check_cap(void);
void web_do_command(void);
void web_root_page(void);
void web_status_page(void);
void web_data_page(void);
void web_notfound(void);
String web_get_request(void);
int web_get_cmd(void);
void web_set_args(void);
void web_set_arg(const char *name, int val);
int web_json_status(char *buff, int maxlen);
void web_dump_args(void);
const char *cmd_str(int cmd);
const char *state_str(int stat);
STATE_VALS get_state(void);
void set_state(STATE_VALS val);
void IRAM_ATTR trig_handler(void);
void set_trig(bool en);

// EOF
