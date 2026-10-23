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

#include "msg_rate.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

int MsgRate::parse(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    if (val == nullptr || storage == nullptr || storage_len < sizeof(std::vector<MsgRateRule>)) {
        return -ENOBUFS;
    }

    auto *target = static_cast<std::vector<MsgRateRule> *>(storage);
    std::string list(val, val_len);
    std::vector<MsgRateRule> rules;

    size_t pos = 0;
    while (pos <= list.size()) {
        size_t end = list.find(',', pos);
        if (end == std::string::npos) {
            end = list.size();
        }
        const std::string token = list.substr(pos, end - pos);
        pos = end + 1;

        // blanks around a token or its parts are allowed; an empty token is not
        const size_t first = token.find_first_not_of(" \t");
        if (first == std::string::npos) {
            return -EINVAL;
        }
        const size_t colon = token.find(':');
        if (colon == std::string::npos) {
            return -EINVAL;
        }

        const std::string id_str = token.substr(first, colon - first);
        const std::string hz_str = token.substr(colon + 1);

        char *stop = nullptr;
        errno = 0;
        const unsigned long id = strtoul(id_str.c_str(), &stop, 10);
        if (errno != 0 || stop == id_str.c_str() || id > 0xffffff
            || id_str.find_first_not_of("0123456789 \t") != std::string::npos) {
            return -EINVAL;
        }
        const double hz = strtod(hz_str.c_str(), &stop);
        if (stop == hz_str.c_str() || hz_str.find_first_not_of("0123456789.eE+- \t")
                                          != std::string::npos
            || !(hz > 0.0) || hz > 1000.0) {
            return -EINVAL;
        }

        rules.push_back({static_cast<uint32_t>(id), static_cast<usec_t>(USEC_PER_SEC / hz)});
    }

    target->insert(target->end(), rules.begin(), rules.end());
    return 0;
}

MsgRate::Verdict MsgRate::offer(uintptr_t link, uint32_t msg_id, uint8_t sysid, uint8_t compid,
                                usec_t interval, usec_t now, const uint8_t *data, size_t len)
{
    const auto key = std::make_tuple(link, msg_id, sysid, compid);
    auto it = _slots.find(key);

    if (it == _slots.end()) {
        _slots[key].last_sent = now;
        return Verdict::Send;
    }

    Slot &s = it->second;

    if (now - s.last_sent >= interval) {
        // a held one is older than this: it is dropped, not sent after it
        s.held = false;
        s.packet.clear();
        s.last_sent = now;
        return Verdict::Send;
    }

    s.held = true;
    s.due = s.last_sent + interval;
    s.packet.assign(data, data + len);
    return Verdict::Held;
}

std::vector<MsgRate::Due> MsgRate::take_due(usec_t now)
{
    std::vector<Due> due;

    for (auto &kv : _slots) {
        Slot &s = kv.second;
        if (s.held && s.due <= now) {
            due.push_back({std::get<0>(kv.first), std::move(s.packet)});
            s.packet.clear();
            s.held = false;
            s.last_sent = now;
        }
    }

    return due;
}

usec_t MsgRate::next_due() const
{
    usec_t next = USEC_INFINITY;

    for (const auto &kv : _slots) {
        if (kv.second.held && kv.second.due < next) {
            next = kv.second.due;
        }
    }

    return next;
}

void MsgRate::forget(uintptr_t link)
{
    for (auto it = _slots.begin(); it != _slots.end();) {
        if (std::get<0>(it->first) == link) {
            it = _slots.erase(it);
        } else {
            ++it;
        }
    }
}
