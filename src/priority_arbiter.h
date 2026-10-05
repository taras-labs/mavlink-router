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
 * Picks one of several ranked sources per key: the best ranked (lowest) one
 * heard within the timeout.
 *
 * - A better rank wins the first time it is heard.
 * - A worse one wins only once everything above it has been silent for the
 *   whole timeout.
 * - A source that loses is still recorded as heard, so the next one in line is
 *   ready the moment the current one goes quiet.
 *
 * Used for component priority (key: target sysid, rank: position of the
 * compid in CompPriority) and master failover (key: vehicle sysid, rank:
 * MasterPriority order).
 */
class PriorityArbiter {
public:
    struct Verdict {
        int winner;   ///< never worse than the rank just heard
        int previous; ///< winner before this call, -1 if none
        bool changed;
    };

    void configure(unsigned ranks, uint32_t timeout_ms);

    unsigned ranks() const { return _ranks; }
    uint32_t timeout_ms() const { return _timeout_us / USEC_PER_MSEC; }

    /* Record that @rank was heard for @key. */
    Verdict observe(int key, unsigned rank, usec_t now);

    /* Best rank heard for @key within the timeout, -1 if none. */
    int winner(int key, usec_t now) const;

private:
    struct State {
        std::vector<usec_t> last_seen; // per rank; 0 = never
        int active = -1;
    };

    bool _fresh(const State &state, unsigned rank, usec_t now) const;

    unsigned _ranks = 0;
    usec_t _timeout_us = 0;
    std::map<int, State> _keys;
};
