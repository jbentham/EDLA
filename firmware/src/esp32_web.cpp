// Web interface for ESP32 logic analyser
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

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
extern "C" {
#include "crypto/base64.h"
}
#include "esp32_la.h"
#include "esp32_web.h"

// Name for mDNS, and Web port number
#define MDNS_NAME       "esp32"
#define HTTP_PORT       80

// Page names
#define STATUS_PAGENAME "/status.txt"
#define DATA_PAGENAME   "/data.txt"

// Web page header to disable cacheing
#define HEADER_NOCACHE  "Cache-Control", "no-cache, no-store, must-revalidate"

// SSID & Password: must be changed!
const char *ssid     = "testnet";
const char *password = "testpass";

uint16_t txbuff[TXBUFF_NSAMP];
extern char version[];

typedef struct {
    char name[16];
    int val;
} SERVER_ARG;

SERVER_ARG server_args[] = {
    {"state",       STATE_IDLE},
    {"nsamp",       0},
    {"xsamp",       10000},
    {"xrate",       100000},
    {"thresh",      THRESH_DEFAULT},
    {"trig_chan",   0},
    {"trig_mode",   0},
    {"trig_pos",    1},
    {""}
};

CAP_PARAMS cap_params;
const char *state_strs[NUM_STATES] = {"Idle", "Ready", "PreLoad", "PreTrig", "PostTrig", "UpLoad"};
const char *cmd_strs[NUM_CMDS]     = {"Stop", "Single", "Multi"};
const char *wifi_states[]          = {"IDLE", "NO SSID", "SCAN_COMPLETE", "CONNECTED",
                                      "CONN FAIL", "CONN LOST", "DISCONNECTED"};
const char *sys_event_reasons[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", 
    "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", 
    "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "UNSPECIFIED",
    "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT",
    "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID",
    "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED",
    "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL",
    "HANDSHAKE_TIMEOUT", "CONNECTION_FAIL" };
WebServer server(HTTP_PORT);
int trigchan, trigflag;
uint32_t startsamp, trigsamp;

// Check network is connected
bool net_check(void) {
    static int lastat=0;
    int stat = WiFi.status();
    if (stat != lastat) {
        if (stat<=WL_DISCONNECTED) {
            DEBUG. printf("WiFi status: %s\r\n", wifi_states[stat]);
            lastat = stat;
        }
        if (stat == WL_DISCONNECTED)
            WiFi.reconnect();
    }
    return(stat == WL_CONNECTED);
}

// Return string for network disconnect reason
const char *net_reason(uint8_t r) {
    return(r>176 ? sys_event_reasons[r-176]: sys_event_reasons[r-1]);
}

// Handle callback for network disconnect
void net_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    uint8_t r = info.disconnected.reason;
    DEBUG.printf("Disconnected: %u %s\r\n", r, net_reason(r));
    DEBUG.printf("Reconnecting to %s\r\n", ssid);
}

// Begin WiFi connection
void net_start(void) {
    DEBUG.print("Connecting to ");
    DEBUG.println(ssid);
    WiFi.onEvent(net_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
}

// Check if WiFi & Web server is ready
bool net_ready(void) {
    bool ok = (WiFi.status() == WL_CONNECTED);
    if (ok) {
        DEBUG.print("Connected, IP ");
        DEBUG.println(WiFi.localIP());
        server.enableCORS();
        server.on("/", web_root_page);
        server.on(STATUS_PAGENAME, web_status_page);
        server.on(DATA_PAGENAME, web_data_page);
        server.onNotFound(web_notfound);
        server.begin();
        if (!MDNS.begin(MDNS_NAME)) {
            DEBUG.println("Error starting mDNS");
        }
        MDNS.addService("http", "tcp", HTTP_PORT);
        DEBUG.println("MDNS name " MDNS_NAME);
        DEBUG.print("HTTP server on port ");
        DEBUG.println(HTTP_PORT);
        delay(100);
    }
    return (ok);
}

// Execute an incoming command from Web browser
void web_do_command(void) {
    CAP_PARAMS *cp = &cap_params;
    int cmd = web_get_cmd();
    int trigchan=server_args[ARG_TRIGCHAN].val;
    int trigmode=server_args[ARG_TRIGMODE].val;
    if (cmd) {
        DEBUG.printf("Command: %s\r\n", cmd_str(cmd));
        cp->xrate = server_args[ARG_XRATE].val;
        cp->xsamp = server_args[ARG_XSAMP].val;
        set_threshold(server_args[ARG_THRESH].val);
        server_args[ARG_NSAMP].val = 0;
        ram_data_sim(0, cp->xsamp);
        startsamp = 0;
        set_state((trigchan && trigmode) ? STATE_PRELOAD : STATE_POSTTRIG);
        cap_start(cp);
    }
}

// Poll Web interface
void web_poll(void) {
    if (server_args[ARG_STATE].val > STATE_READY) {
        web_check_cap();
    }
    server.handleClient();
}

// Check progress of capture, return non-zero if complete
bool web_check_cap(void) {
    uint32_t nsamp = pcnt_val32(), xsamp = server_args[ARG_XSAMP].val;
    uint32_t presamp = (xsamp/10) * server_args[ARG_TRIGPOS].val;
    STATE_VALS state = (STATE_VALS)server_args[ARG_STATE].val;
    server_args[ARG_NSAMP].val = nsamp;
    if (state == STATE_PRELOAD) {
        if (nsamp > presamp)
            set_state(STATE_PRETRIG);
    }
    else if (state == STATE_PRETRIG) {
        if (trigflag) {
            startsamp = trigsamp - presamp;
            set_state(STATE_POSTTRIG);
        }
    }
    else if (state == STATE_POSTTRIG) {
        if (nsamp-startsamp > xsamp) {
            cap_end();
            set_state(STATE_READY);
            //DEBUG.printf("Captured: startsamp %u, trigsamp %u, nsamp %u\r\n", 
            //    startsamp, trigsamp, nsamp);
            return(true);
        }
    }
    return (false);
}

// Return root Web page
void web_root_page(void) {
    server.sendHeader(HEADER_NOCACHE);
    sprintf((char *)txbuff, "%s, attenuator %u:1", version, THRESH_SCALE);
    server.send(200, "text/plain", (char *)txbuff);
}

// Return status Web page
void web_status_page(void) {
    DEBUG.println(web_get_request());
    web_set_args();
    web_do_command();
    web_json_status((char *)txbuff, TXBUFF_LEN);
    server.sendHeader(HEADER_NOCACHE);
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json");
    server.sendContent((char *)txbuff);
    server.sendContent("");
    DEBUG.println((char *)txbuff);
}

// Return data Web page
void web_data_page(void) {
    DEBUG.println(web_get_request());
    web_set_args();
    web_do_command();
    server.sendHeader(HEADER_NOCACHE);
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain");
    cap_read_start(startsamp);
    int count=0, nsamp=server_args[ARG_XSAMP].val;
    size_t outlen = 0;
    while (count < nsamp) {
        size_t n = min(nsamp - count, TXBUFF_NSAMP);
        cap_read_block(txbuff, n);
        byte *enc = base64_encode((byte *)txbuff, n * 2, &outlen);
        count += n;
        server.sendContent((char *)enc);
        free(enc);
    }
    server.sendContent("");
    cap_read_end();
    web_json_status((char *)txbuff, TXBUFF_LEN);
    DEBUG.println((char *)txbuff);
}

// Handler for page not found
void web_notfound(void) {
    sprintf((char *)txbuff, "Content not found: %s", server.uri().c_str());
    server.send(404, "text/plain", (char *)txbuff);
};

// Get HTTP request
String web_get_request(void) {
  static String req;
  req = server.method() == HTTP_GET ? "GET " : "POST ";
  req += server.uri();
  for (int i = 0; i < server.args(); i++) {
      req += " " + server.argName(i) + "=" + server.arg(i) ;
  }
  return(req);
}

// Get command from incoming Web request
int web_get_cmd(void) {
    for (int i=0; i<server.args(); i++) {
        if (!strcmp(server.argName(i).c_str(), "cmd"))
            return(atoi(server.arg(i).c_str()));
    }
    return(0);
}

// Get arguments from incoming Web request
void web_set_args(void) {
    for (int i=0; i<server.args(); i++) {
        int val = atoi(server.arg(i).c_str());
        web_set_arg(server.argName(i).c_str(), val);
    }
}

// Set argument value
void web_set_arg(const char *name, int val) {
    SERVER_ARG *arg = server_args;
    if (strlen(name) < sizeof(arg->name)) {
        while (arg->name[0]) {
            if (!(strcmp(name, arg->name))) {
                arg->val = val;
                break;
            }
            arg++;
        }
    }
}

// Return server status as json string
int web_json_status(char *buff, int maxlen) {
    SERVER_ARG *arg = server_args;
    int n=sprintf(buff, "{");
    while (arg->name[0] && n<maxlen-20) {
        n += sprintf(&buff[n], "%s\"%s\":%d", n>2?",":"", arg->name, arg->val);
        arg++;
    }
    return(n += sprintf(&buff[n], "}"));
}

// Dump server arguments
void web_dump_args(void) {
    SERVER_ARG *arg = server_args;
    while (arg->name[0]) {
        DEBUG.printf("%s=%d ", arg->name, arg->val);
        arg++;
    }
    DEBUG.println("");
}

// Return string for a command
const char *cmd_str(int cmd) {
    return (cmd>=0 && cmd<NUM_CMDS ? cmd_strs[cmd] : "");
}
// Return string for a state value
const char *state_str(int stat) {
    return (stat>=0 && stat<NUM_STATES ? state_strs[stat] : "");
}

// Get the current state
STATE_VALS get_state(void) {
    return((STATE_VALS)server_args[ARG_STATE].val);
}
// Set the current state
void set_state(STATE_VALS val) {
    STATE_VALS last = (STATE_VALS)server_args[ARG_STATE].val;
    server_args[ARG_STATE].val = val;
    if (last==STATE_IDLE || last==STATE_READY) {
        startsamp = trigsamp = 0;
        set_trig(0);
    }
    else if (val == STATE_PRETRIG)
        set_trig(1);
    if (val < NUM_STATES)
        DEBUG.printf("State: %s\r\n", state_strs[val]);
}

// Handler for trigger interrupt
void IRAM_ATTR trig_handler(void) {
    if (!trigflag) {
        trigsamp = pcnt_val32();
        trigflag = 1;
    }
}

// Enable or disable the trigger interrupt for channels 1 to 16
void set_trig(bool en) {
    int chan=server_args[ARG_TRIGCHAN].val, mode=server_args[ARG_TRIGMODE].val;
    if (trigchan) {
        detachInterrupt(busbit_pin(trigchan-1));
        trigchan = 0;
    }
    if (en && chan && mode) {
        attachInterrupt(busbit_pin(chan-1), trig_handler, 
            mode==TRIG_FALLING ? FALLING : RISING);
        trigchan = chan;
    }
    trigflag = 0;
}

// EOF
