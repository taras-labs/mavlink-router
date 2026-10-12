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

#include <cstddef>
#include <cstdint>
#include <vector>

#include <common/mavlink.h>

/*
 * Keeps chosen messages switched off on autopilots.
 *
 * Each one is sent MAV_CMD_SET_MESSAGE_INTERVAL with an interval of -1 ("stop
 * sending"). The autopilot keeps that in RAM only, and per link: a reboot, or
 * any GCS's REQUEST_DATA_STREAM, turns the message back on. So the commands
 * are re-sent for as long as the router runs.
 *
 * One message per tick, round robin, rather than the whole list at once: on a
 * slow radio a burst of commands queues in the uplink in front of the RC
 * overrides, and one small command a tick never does.
 */
class MessageDisable {
public:
    MessageDisable() = default;

    /*
     * @msg_ids: messages to keep off. Empty disables the feature.
     */
    void configure(std::vector<uint32_t> msg_ids);

    bool enabled() const { return !_msg_ids.empty(); }
    size_t size() const { return _msg_ids.size(); }

    /*
     * The message to send the command for on this tick; moves on to the next.
     * Only valid while enabled().
     */
    uint32_t next();

    /*
     * Packs the command from sysid/compid for the autopilot at
     * target_sys_comp_id (sysid << 8 | compid).
     */
    static void pack(uint8_t sysid, uint8_t compid, uint16_t target_sys_comp_id,
                     uint32_t msg_id, mavlink_message_t *msg);

private:
    std::vector<uint32_t> _msg_ids;
    size_t _next = 0;
};
