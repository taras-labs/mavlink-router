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
#include <map>
#include <tuple>
#include <vector>

#include <common/mavlink.h>
#include <common/util.h>

/*
 * Keeps chosen messages switched off on autopilots.
 *
 * Each one is sent MAV_CMD_SET_MESSAGE_INTERVAL with an interval of -1 ("stop
 * sending"). The autopilot keeps that in RAM only, and per link: a reboot, or
 * any GCS's REQUEST_DATA_STREAM, turns the message back on. So the command is
 * sent when a listed message is heard, on the link it was heard on, and never
 * while the autopilot keeps them off: a vehicle that obeys costs nothing.
 *
 * At most one command per link per SPACING: after a reboot every listed
 * message shows up at once, and on a slow radio a burst of commands queues in
 * the uplink in front of the RC overrides. A message refused for spacing is
 * commanded on one of its next copies. And one command per message per
 * HOLDOFF: copies the autopilot sent before it obeyed are still coming down a
 * queued radio (a second and more on a SineLink), and each would be answered
 * with another command; a command lost on the way is re-sent after it.
 */
class MessageDisable {
public:
    static constexpr usec_t SPACING = USEC_PER_SEC;
    static constexpr usec_t HOLDOFF = 3 * USEC_PER_SEC;

    MessageDisable() = default;

    /*
     * @msg_ids: messages to keep off. Empty disables the feature.
     */
    void configure(std::vector<uint32_t> msg_ids);

    bool enabled() const { return !_msg_ids.empty(); }
    size_t size() const { return _msg_ids.size(); }
    bool listed(uint32_t msg_id) const;

    /*
     * @msg_id was heard from the autopilot @sys_comp_id (sysid << 8 | compid)
     * on @link at @now. True when the command for it is to be sent on that
     * link now; it is then counted as sent.
     */
    bool heard(uintptr_t link, uint16_t sys_comp_id, uint32_t msg_id, usec_t now);

    /*
     * Packs the command from sysid/compid for the autopilot at
     * target_sys_comp_id (sysid << 8 | compid).
     */
    static void pack(uint8_t sysid, uint8_t compid, uint16_t target_sys_comp_id,
                     uint32_t msg_id, mavlink_message_t *msg);

private:
    std::vector<uint32_t> _msg_ids;
    std::map<std::tuple<uintptr_t, uint16_t, uint32_t>, usec_t> _sent; // per message
    std::map<uintptr_t, usec_t> _link_sent;                           // per link
};
