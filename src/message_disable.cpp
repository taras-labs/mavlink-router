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

#include "message_disable.h"

#include <algorithm>
#include <utility>

void MessageDisable::configure(std::vector<uint32_t> msg_ids)
{
    _msg_ids = std::move(msg_ids);
    _sent.clear();
    _link_sent.clear();
}

bool MessageDisable::listed(uint32_t msg_id) const
{
    return std::find(_msg_ids.begin(), _msg_ids.end(), msg_id) != _msg_ids.end();
}

bool MessageDisable::heard(uintptr_t link, uint16_t sys_comp_id, uint32_t msg_id, usec_t now)
{
    if (!listed(msg_id)) {
        return false;
    }

    const auto key = std::make_tuple(link, sys_comp_id, msg_id);
    auto sent = _sent.find(key);
    if (sent != _sent.end() && now - sent->second < HOLDOFF) {
        return false;
    }

    auto link_sent = _link_sent.find(link);
    if (link_sent != _link_sent.end() && now - link_sent->second < SPACING) {
        return false;
    }

    _sent[key] = now;
    _link_sent[link] = now;
    return true;
}

void MessageDisable::pack(uint8_t sysid, uint8_t compid, uint16_t target_sys_comp_id,
                          uint32_t msg_id, mavlink_message_t *msg)
{
    mavlink_msg_command_long_pack(sysid,
                                  compid,
                                  msg,
                                  target_sys_comp_id >> 8,
                                  target_sys_comp_id & 0xff,
                                  MAV_CMD_SET_MESSAGE_INTERVAL,
                                  0,
                                  (float)msg_id,
                                  -1, // stop sending
                                  0,
                                  0,
                                  0,
                                  0,
                                  0);
}
