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

#include "comp_priority.h"

#include <algorithm>
#include <utility>

#include <common/log.h>

void CompPriority::configure(std::vector<uint8_t> compids, std::vector<uint32_t> msg_ids,
                             uint32_t timeout_ms)
{
    _compids = std::move(compids);
    _msg_ids = std::move(msg_ids);
    _timeout_us = (usec_t)timeout_ms * USEC_PER_MSEC;
    _targets.clear();
}

bool CompPriority::check(uint32_t msg_id, int target_sysid, uint8_t src_compid, usec_t now)
{
    if (!enabled()) {
        return true;
    }

    if (std::find(_msg_ids.begin(), _msg_ids.end(), msg_id) == _msg_ids.end()) {
        return true;
    }

    auto it = std::find(_compids.begin(), _compids.end(), src_compid);
    if (it == _compids.end()) {
        return true;
    }
    const int rank = it - _compids.begin();

    TargetState &state = _targets[target_sysid];
    state.last_seen.resize(_compids.size(), 0);
    state.last_seen[rank] = now;

    // The sender itself was seen just now, so a winner always exists and
    // never ranks below it.
    int winner = rank;
    for (int i = 0; i < rank; i++) {
        if (state.last_seen[i] != 0 && now - state.last_seen[i] <= _timeout_us) {
            winner = i;
            break;
        }
    }

    if (winner != state.active) {
        if (state.active < 0) {
            log_info("Comp priority: target %d driven by compid %u", target_sysid,
                     _compids[winner]);
        } else if (winner < state.active) {
            log_info("Comp priority: target %d taken over by compid %u from %u", target_sysid,
                     _compids[winner], _compids[state.active]);
        } else {
            log_info("Comp priority: target %d falls back to compid %u, %u silent for %u ms",
                     target_sysid, _compids[winner], _compids[state.active],
                     (unsigned)(_timeout_us / USEC_PER_MSEC));
        }
        state.active = winner;
    }

    return winner == rank;
}
