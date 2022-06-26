# Fetch & decode data from EDLA logic analyser units
# For detailed description, see https://iosoft.blog/edla
#
# Copyright (c) 2022 Jeremy P Bentham
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# v0.05 JPB 26/6/22 Tidied up for release

import urllib.request, json, time, sys, base64, struct

# Remote unit states
unit_states = "IDLE", "READY", "UNTRIG", "PRETRIG", "POSTTRIG"

# Command values
CMD_STOP, CMD_SINGLE = 0, 1

# Flag to enable verbose display
verbose = True

class EdlaUnit(object):
    def __init__(self, unit_num, subnet):
        self.unit_num = unit_num
        self.params = {
            'thresh':  1500,    # Logic threshold
            'xsamp':   20000,   # Sample count
            'xrate':   10000,   # Rate in samples/sec
        }
        self.base_url   = "http://%s.%u/" % (subnet, unit_num+10)
        self.status_url = self.base_url + "status.txt"
        self.data_url   = self.base_url + "data.txt"
        self.page = None

    # Fetch page, given URL
    def fetch_page(self, url, timeout=2):
        try:
            page = urllib.request.urlopen(url, timeout=timeout)
            data = page.read().decode("UTF-8")
        except:
            data = None
        return data

    # Fetch unit status page, sending parameters with optional command
    # Return values in dictionay, or None if failed
    def fetch_status(self, cmd=None, timeout=2):
        param_list = '&'.join('{}={}'.format(k, v) for k, v in self.params.items())
        if cmd != None:
            param_list += "&cmd=%u" % cmd
        url = self.status_url + '?' + param_list
        resp = self.fetch_page(url, timeout)
        if not resp:
            resp = self.fetch_page(url, timeout)
        return json.loads(resp) if resp else None
    
    # Check status response to see if unit has captured the required data
    # Return False if capture in progress, or None if capture failed
    def status_got_samp(self, status):
        return True if status['nsamp']>=status['xsamp'] else None if status['state']<2 else False

    # Start a capture, return True if OK
    def start_capture(self):
        return self.fetch_status(CMD_STOP) and self.fetch_status(CMD_SINGLE)
        
    # Check status of capture, return True if complete, None if error
    def check_capture(self):
        status = self.fetch_status()
        return None if status==None else self.status_got_samp(status)
        
    # Do a capture
    def do_capture(self):
        cap = None
        if not self.start_capture():
            write_stdout("Can't start capture")
        else:
            write_stdout("Capturing.")
            cap = self.check_capture()
            while cap == False:
                write_stdout(".")
                time.sleep(1)
                cap = self.check_capture()
            write_stdout("OK\n" if cap else "ERROR\n")
        return cap == True

    # Start loading capture data
    def start_load(self):
        self.page = None
        try:
            self.page = urllib.request.urlopen(self.data_url, timeout=5)
            data = self.page.read(10000).decode("UTF-8")
        except:
            data = None
        return data
        
    # Poll for more captured data
    def next_load(self):
        try:
            data = self.page.read(10000).decode("UTF-8")
        except:
            data = None
        return data
        
    # Load captured data
    def do_load(self):
        write_stdout("Loading.")
        data = self.start_load()
        if not data:
            write_stdout(" FAILED")
        while data:
            write_stdout(".")
            d = self.next_load()
            if d:
                data += d
            else:
                ok = d != None
                write_stdout("OK\n" if ok else "ERROR\n")
                break
        if self.page:
            self.page.close()
        return data
        
    # Set threshold voltage
    def set_threshold(self, v):
        self.params['thresh'] = v
    # Set sample count
    def set_sample_count(self, n):
        self.params['xsamp'] = n
    # Set sample rate
    def set_sample_rate(self, n):
        self.params['xrate'] = n

# Convert base-64 data into tuple of 16-bit words
def base2words(b64):
    byts = base64.b64decode(b64)
    return [w[0] for w in struct.iter_unpack('<H', byts)]
    
# Write string to stdout, if in 'verbose' mode
def write_stdout(s):
    if verbose:
        sys.stdout.write(s)
        sys.stdout.flush()

# Enable or disable 'verbose' mode
def verbose_mode(b):
    global verbose
    verbose = b
    
if __name__ == "__main__":
    unit = EdlaUnit(1, "192.168.8")
    ok = False
    data = None
    status = unit.fetch_status()
    if status:
        ok = unit.do_capture()
    else:
        print("Can't fetch status from %s" % unit.status_url)
    if ok:
        data = unit.do_load()
    if data == None:
        print("Can't load data")
    else:
        samps = base2words(data)
        print("Loaded %u samples" % len(samps))

# EOF
