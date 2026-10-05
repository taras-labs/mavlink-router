/*
 * This file is part of the MAVLink Router project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include <common/util.h>

/*
 * Source-component arbitration for control messages.
 *
 * Several components may drive the same vehicle with the same kind of
 * message (two RC transmitters both sending RC_CHANNELS_OVERRIDE, say). The
 * autopilot takes whichever frame arrived last, so the sticks fight. This
 * class lets exactly one of them through: the highest-priority component that
 * has sent one of the arbitrated messages within the timeout.
 *
 * - A higher-priority component takes over on its first frame.
 * - A lower-priority one takes over only after the higher one has been silent
 *   for the whole timeout.
 * - Dropped frames still count as "seen", so the next component in line is
 *   ready to take over the moment the current one goes quiet.
 * - Components and messages not listed are never touched.
 * - Arbitration is per target system: two vehicles never share a winner.
 */
class CompPriority {
public:
    CompPriority() = default;

    /*
     * @compids: highest priority first. Empty disables arbitration.
     * @msg_ids: messages to arbitrate.
     * @timeout_ms: silence after which a component loses its turn.
     */
    void configure(std::vector<uint8_t> compids, std::vector<uint32_t> msg_ids,
                   uint32_t timeout_ms);

    bool enabled() const { return !_compids.empty() && !_msg_ids.empty(); }

    /*
     * Returns false if the message must be dropped. Records the sighting
     * either way.
     */
    bool check(uint32_t msg_id, int target_sysid, uint8_t src_compid, usec_t now);

private:
    struct TargetState {
        std::vector<usec_t> last_seen; // per rank; 0 = never
        int active = -1;               // rank currently let through
    };

    std::vector<uint8_t> _compids;
    std::vector<uint32_t> _msg_ids;
    usec_t _timeout_us = 0;
    std::map<int, TargetState> _targets;
};
