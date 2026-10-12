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

#include <utility>

void MessageDisable::configure(std::vector<uint32_t> msg_ids)
{
    _msg_ids = std::move(msg_ids);
    _next = 0;
}

uint32_t MessageDisable::next()
{
    uint32_t msg_id = _msg_ids[_next];
    _next = (_next + 1) % _msg_ids.size();
    return msg_id;
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
