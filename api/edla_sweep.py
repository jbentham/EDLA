# Sweep the threshold voltage of EDLA remote logic analyser
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
# v0.04 JPB 26/6/22 Tidied up for release

import edla_utils as edla, base64, numpy as np

edla.verbose_mode(False)
unit = edla.EdlaUnit(1, "192.168.8")
unit.set_sample_rate(10000)
unit.set_sample_count(10000)

MIN_V, MAX_V, STEP_V = 0, 51, 5

def get_data():
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
    return data

for v in range(MIN_V, MAX_V, STEP_V):
    unit.set_threshold(v)
    d = get_data()
    byts = base64.b64decode(d)
    samps = np.frombuffer(byts, dtype=np.uint16)
    # print("Loaded %u samples" % len(samps))
    diffs = np.diff(samps)
    edges = np.where(diffs != 0)[0]
    # print("%u edges" % len(edges))
    totals = np.zeros(16, dtype=int)
    for edge in edges:
        bits = samps[edge] ^ samps[edge+1]
        for n in range(0, 15):
            if bits & (1<<n):
                totals[n] += 1
    s = "%4u," % v
    s += ",".join([("%4u" % val) for val in totals])
    print(s)

# EOF
