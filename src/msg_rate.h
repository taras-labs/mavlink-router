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

#include <common/util.h>

/*
 * MsgRateOut = <msg id>:<Hz>[, ...] on an endpoint: that message leaves the
 * endpoint at that rate at most.
 *
 * Per message id, and per sender (sysid/compid) of it, towards that endpoint:
 *  - one that arrives when its interval since the last one sent has passed is
 *    sent at once;
 *  - one that arrives sooner is held until the interval is over;
 *  - a fresher one arriving meanwhile replaces the held one, which is then
 *    never sent: what leaves is always the latest value, never a stale one.
 * Nothing is sent twice and nothing is sent that did not arrive.
 */
struct MsgRateRule {
    uint32_t msg_id;
    usec_t interval; ///< between two messages sent, 1 / Hz
};

class MsgRate {
public:
    /*
     * ConfFile option parser: "30:10, 33:2.5" appended to the
     * std::vector<MsgRateRule> at @storage. -EINVAL on anything else, a rate
     * of 0 included (BlockMsgIdOut is how a message is stopped).
     */
    static int parse(const char *val, size_t val_len, void *storage, size_t storage_len);

    enum class Verdict {
        Send, ///< send it now
        Held, ///< kept; take_due() hands it out when its interval is over
    };

    /*
     * A message came in for @link (an endpoint, as an opaque key), allowed
     * @interval between two. @data/@len is the packet as it is to be written.
     */
    Verdict offer(uintptr_t link, uint32_t msg_id, uint8_t sysid, uint8_t compid,
                  usec_t interval, usec_t now, const uint8_t *data, size_t len);

    struct Due {
        uintptr_t link;
        std::vector<uint8_t> packet;
    };

    /*
     * The held messages whose interval is over at @now. Each is counted as
     * sent at @now.
     */
    std::vector<Due> take_due(usec_t now);

    /* When the first held message is due, USEC_INFINITY when none is held. */
    usec_t next_due() const;

    void forget(uintptr_t link);
    void clear() { _slots.clear(); }

private:
    struct Slot {
        usec_t last_sent = 0;
        usec_t due = 0;
        bool held = false;
        std::vector<uint8_t> packet;
    };

    std::map<std::tuple<uintptr_t, uint32_t, uint8_t, uint8_t>, Slot> _slots;
};
