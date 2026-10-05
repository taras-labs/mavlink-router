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

#include "priority_arbiter.h"

void PriorityArbiter::configure(unsigned ranks, uint32_t timeout_ms)
{
    _ranks = ranks;
    _timeout_us = (usec_t)timeout_ms * USEC_PER_MSEC;
    _keys.clear();
}

bool PriorityArbiter::_fresh(const State &state, unsigned rank, usec_t now) const
{
    return rank < state.last_seen.size() && state.last_seen[rank] != 0
        && now - state.last_seen[rank] <= _timeout_us;
}

PriorityArbiter::Verdict PriorityArbiter::observe(int key, unsigned rank, usec_t now)
{
    State &state = _keys[key];
    state.last_seen.resize(_ranks, 0);
    state.last_seen[rank] = now;

    // The rank itself was heard just now, so a winner always exists and never
    // ranks below it.
    int winner = rank;
    for (unsigned i = 0; i < rank; i++) {
        if (_fresh(state, i, now)) {
            winner = i;
            break;
        }
    }

    Verdict verdict{winner, state.active, winner != state.active};
    state.active = winner;
    return verdict;
}

int PriorityArbiter::winner(int key, usec_t now) const
{
    auto it = _keys.find(key);
    if (it == _keys.end()) {
        return -1;
    }

    for (unsigned i = 0; i < _ranks; i++) {
        if (_fresh(it->second, i, now)) {
            return i;
        }
    }
    return -1;
}
