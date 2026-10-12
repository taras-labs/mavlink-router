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

#include "rc_gate.h"

#include <arpa/inet.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <utility>

#include <common/mavlink.h>

// Out-of-line definitions, for code that takes their address (a gtest
// EXPECT_EQ does) under C++14.
constexpr int32_t RcGate::LAST;
constexpr size_t RcGate::CHANNELS;

void RcGate::configure(std::vector<uint8_t> pass_compids, uint32_t failsafe_timeout_ms,
                       const Template &failsafe)
{
    _enabled = true;
    _pass = std::move(pass_compids);
    _timeout_ms = failsafe_timeout_ms;
    _failsafe = failsafe;
}

void RcGate::bind_endpoints(std::string pilot_endpoint, std::string local_endpoint,
                            std::string priority_endpoint)
{
    _pilot_endpoint = std::move(pilot_endpoint);
    _local_endpoint = std::move(local_endpoint);
    _priority_endpoint = std::move(priority_endpoint);
}

bool RcGate::is_rc(uint32_t msg_id)
{
    return msg_id == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE || msg_id == MAVLINK_MSG_ID_MANUAL_CONTROL;
}

// TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
bool RcGate::patch_sysid(uint8_t *frame, unsigned int len, uint8_t sysid)
{
    unsigned int header_len, sysid_at;
    uint32_t msg_id;
    if (len >= 10 && frame[0] == MAVLINK_STX) {
        if (frame[2] & MAVLINK_IFLAG_SIGNED) {
            return false;
        }
        header_len = 10;
        sysid_at = 5;
        msg_id = frame[7] | (frame[8] << 8) | ((uint32_t)frame[9] << 16);
    } else if (len >= 6 && frame[0] == MAVLINK_STX_MAVLINK1) {
        header_len = 6;
        sysid_at = 3;
        msg_id = frame[5];
    } else {
        return false;
    }
    const unsigned int payload_len = frame[1];
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msg_id);
    if (entry == nullptr || len < header_len + payload_len + 2) {
        return false;
    }

    frame[sysid_at] = sysid;
    // The checksum covers the header but its magic byte, the payload and the
    // message's crc_extra.
    uint16_t crc = crc_calculate(&frame[1], header_len + payload_len - 1);
    crc_accumulate(entry->crc_extra, &crc);
    frame[header_len + payload_len] = crc & 0xff;
    frame[header_len + payload_len + 1] = crc >> 8;
    return true;
}

void RcGate::set_priority(uint8_t compid, uint32_t timeout_ms)
{
    _has_priority = true;
    _priority = compid;
    _priority_timeout_ms = timeout_ms;
}

// Whether a channel value overrides the channel: anything but "leave it"
// (UINT16_MAX, and 0 on 9-18) and "release" (0 on 1-8, UINT16_MAX-1 on 9-18).
static bool overrides(size_t channel, uint16_t v)
{
    return channel < 8 ? (v != 0 && v != UINT16_MAX)
                       : (v != 0 && v != UINT16_MAX && v != UINT16_MAX - 1);
}

static std::array<uint16_t, RcGate::CHANNELS> decode(const uint8_t *payload, uint8_t len)
{
    // MAVLink 2 trims the payload's trailing zeros: the zero-filled struct puts
    // them back.
    mavlink_rc_channels_override_t ov = {};
    memcpy(&ov, payload, std::min<size_t>(len, sizeof(ov)));
    return {ov.chan1_raw,  ov.chan2_raw,  ov.chan3_raw,  ov.chan4_raw,  ov.chan5_raw,
            ov.chan6_raw,  ov.chan7_raw,  ov.chan8_raw,  ov.chan9_raw,  ov.chan10_raw,
            ov.chan11_raw, ov.chan12_raw, ov.chan13_raw, ov.chan14_raw, ov.chan15_raw,
            ov.chan16_raw, ov.chan17_raw, ov.chan18_raw};
}

static std::string trim(const std::string &s)
{
    const char *ws = " \t\r\n";
    const auto begin = s.find_first_not_of(ws);
    if (begin == std::string::npos) {
        return "";
    }
    return s.substr(begin, s.find_last_not_of(ws) - begin + 1);
}

bool RcGate::parse_mode(const std::string &text, Mode *mode, std::string *pilot_ip)
{
    std::string word = trim(text);
    std::string arg;
    const auto space = word.find_first_of(" \t");
    if (space != std::string::npos) {
        arg = trim(word.substr(space));
        word = word.substr(0, space);
    }
    if (!arg.empty()) {
        // an address, and only with "pilot"
        struct in6_addr a6;
        struct in_addr a4;
        if (word != "pilot"
            || (inet_pton(AF_INET, arg.c_str(), &a4) != 1
                && inet_pton(AF_INET6, arg.c_str(), &a6) != 1)) {
            return false;
        }
    }
    pilot_ip->assign(arg);

    if (word == "open") {
        *mode = Mode::Open;
    } else if (word == "pilot") {
        *mode = Mode::Pilot;
    } else if (word == "local") {
        *mode = Mode::Local;
    } else {
        return false;
    }
    return true;
}

const char *RcGate::mode_name(Mode mode)
{
    switch (mode) {
    case Mode::Open:
        return "open";
    case Mode::Pilot:
        return "pilot";
    case Mode::Local:
        return "local";
    }
    return "?";
}

bool RcGate::parse_failsafe(const std::string &text, Template *out)
{
    Template t;
    t.fill(LAST);

    size_t channel = 0;
    size_t pos = 0;
    const std::string all = trim(text);
    while (!all.empty() && pos <= all.size()) {
        size_t comma = all.find(',', pos);
        if (comma == std::string::npos) {
            comma = all.size();
        }
        const std::string tok = trim(all.substr(pos, comma - pos));
        if (channel >= CHANNELS) {
            return false;
        }
        if (tok == "last") {
            t[channel] = LAST;
        } else if (tok == "ignore") {
            t[channel] = UINT16_MAX;
        } else {
            char *end = nullptr;
            const long v = tok.empty() ? -1 : strtol(tok.c_str(), &end, 10);
            if (tok.empty() || *end != '\0' || v < 0 || v > UINT16_MAX) {
                return false;
            }
            t[channel] = (int32_t)v;
        }
        channel++;
        pos = comma + 1;
    }

    *out = t;
    return true;
}

bool RcGate::set_mode(Mode mode, usec_t now, const std::string &pilot_ip)
{
    _pilot_ip = pilot_ip;
    if (mode == _mode) {
        return false;
    }
    _mode = mode;
    if (mode == Mode::Pilot) {
        _pilot_seen = now;
    }
    return true;
}

bool RcGate::check(uint32_t msg_id, uint8_t src_compid, const uint8_t *payload,
                   uint8_t payload_len, usec_t now, const std::string &endpoint,
                   const std::string &peer)
{
    if (!_enabled || !is_rc(msg_id)) {
        return true;
    }
    const Source source = _source(src_compid, endpoint);
    if (source == Source::Pass) {
        const bool passes = _pass_allowed(now);
        if (passes == _pass_blocked) {
            _pass_blocked = !passes;
            _pass_changed = true;
        }
        return passes;
    }

    const bool passes = _check_sticks(source, msg_id, payload, payload_len, now, peer);
    if (passes) {
        _stick_seen = now;
    }
    return passes;
}

bool RcGate::_pass_allowed(usec_t now) const
{
    if (_pass_timeout_ms == 0 || _stick_seen == 0) {
        return true; // not asked for, or nobody has flown by RC yet
    }
    // Stick frames from a sender only: the router's own failsafe frame does not
    // count, so nothing re-sent keeps the FC's radio alive.
    return now - _stick_seen < (usec_t)_pass_timeout_ms * USEC_PER_MSEC;
}

bool RcGate::take_pass_change(bool *blocked)
{
    if (!_pass_changed) {
        return false;
    }
    _pass_changed = false;
    *blocked = _pass_blocked;
    return true;
}

RcGate::Source RcGate::_source(uint8_t src_compid, const std::string &endpoint) const
{
    // The two transmitters by their port alone, before any compid is looked
    // at: what the pilot's crsf2mav writes in its header cannot make it the
    // priority component or an always-pass one.
    if (!_pilot_endpoint.empty() && endpoint == _pilot_endpoint) {
        return Source::Pilot;
    }
    if (!_local_endpoint.empty() && endpoint == _local_endpoint) {
        return Source::Local;
    }
    if (std::find(_pass.begin(), _pass.end(), src_compid) != _pass.end()) {
        return Source::Pass;
    }
    if (_has_priority && src_compid == _priority
        && (_priority_endpoint.empty() || endpoint == _priority_endpoint)) {
        return Source::Priority;
    }
    return Source::Other;
}

bool RcGate::_check_sticks(Source source, uint32_t msg_id, const uint8_t *payload,
                           uint8_t payload_len, usec_t now, const std::string &peer)
{
    const bool rc_frame = msg_id == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE && payload != nullptr;

    if (source == Source::Priority) {
        bool holds = !rc_frame; // MANUAL_CONTROL always drives
        if (rc_frame) {
            const auto ch = decode(payload, payload_len);
            for (size_t i = 0; i < CHANNELS && !holds; i++) {
                holds = overrides(i, ch[i]);
            }
        }
        _priority_seen = holds ? now : 0;
        if (_mode == Mode::Pilot) {
            _pilot_seen = now;
        }
        return true;
    }

    const bool from_pilot = source == Source::Pilot
        && (_mode != Mode::Pilot || _pilot_ip.empty() || peer == _pilot_ip);
    if (from_pilot && rc_frame) {
        _pilot_last = decode(payload, payload_len);
        _have_last = true;
    }

    if (_priority_seen != 0
        && now - _priority_seen < (usec_t)_priority_timeout_ms * USEC_PER_MSEC) {
        return false; // the priority component holds the sticks
    }

    switch (_mode) {
    case Mode::Open:
        return true;
    case Mode::Pilot:
        if (from_pilot) {
            _pilot_seen = now;
        }
        return from_pilot;
    case Mode::Local:
        return _local_endpoint.empty() ? source != Source::Pilot : source == Source::Local;
    }
    return true;
}

bool RcGate::failsafe_due(usec_t now) const
{
    return _enabled && _mode == Mode::Pilot && _timeout_ms > 0
        && now - _pilot_seen >= (usec_t)_timeout_ms * USEC_PER_MSEC;
}

void RcGate::failsafe_channels(uint16_t out[CHANNELS]) const
{
    for (size_t i = 0; i < CHANNELS; i++) {
        if (_failsafe[i] == LAST) {
            // Nothing from the pilot yet: leave the channel as it is.
            out[i] = _have_last ? _pilot_last[i] : UINT16_MAX;
        } else {
            out[i] = (uint16_t)_failsafe[i];
        }
    }
}

bool RcGate::release_channels(bool after_failsafe, uint16_t out[CHANNELS]) const
{
    uint16_t held[CHANNELS];
    if (after_failsafe) {
        failsafe_channels(held);
    } else if (_have_last) {
        std::copy(_pilot_last.begin(), _pilot_last.end(), held);
    } else {
        return false;
    }

    bool any = false;
    for (size_t i = 0; i < CHANNELS; i++) {
        const bool overrode = overrides(i, held[i]);
        out[i] = overrode ? (i < 8 ? 0 : UINT16_MAX - 1) : UINT16_MAX;
        any = any || overrode;
    }
    return any;
}
