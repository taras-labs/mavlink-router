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
    _arbiter.configure(_compids.size(), timeout_ms);
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

    auto v = _arbiter.observe(target_sysid, rank, now);
    if (v.changed) {
        if (v.previous < 0) {
            log_info("Comp priority: target %d driven by compid %u", target_sysid,
                     _compids[v.winner]);
        } else if (v.winner < v.previous) {
            log_info("Comp priority: target %d taken over by compid %u from %u", target_sysid,
                     _compids[v.winner], _compids[v.previous]);
        } else {
            log_info("Comp priority: target %d falls back to compid %u, %u silent for %u ms",
                     target_sysid, _compids[v.winner], _compids[v.previous],
                     _arbiter.timeout_ms());
        }
    }

    return v.winner == rank;
}
