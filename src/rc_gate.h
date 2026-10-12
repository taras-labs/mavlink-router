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

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <common/util.h>

/*
 * Who may drive the vehicle's sticks, by remote-pilot session.
 *
 * The two transmitters are told apart by the endpoint their frames arrive on,
 * never by the component id in the header: a compid is whatever the sender
 * writes, so it names a source in the logs and identifies nothing. Whatever
 * arrives on the pilot's endpoint is the remote pilot's, whatever arrives on
 * the local endpoint is this station's own transmitter's, whichever compid
 * either writes.
 *
 * Whoever owns the session (tt-monitor, from what tt-admin tells it) sets one
 * of three modes over the RcControlPort socket:
 *
 * - open:  no remote pilot. RC passes untouched, whoever sends it.
 * - pilot: a remote pilot holds the station. RC_CHANNELS_OVERRIDE and
 *          MANUAL_CONTROL pass only from the pilot's endpoint, and from the
 *          always-pass components (the digital link's freq/quality frame).
 *          While the pilot has sent nothing for the failsafe timeout, the
 *          router sends a failsafe frame of its own instead: every channel
 *          overridden, so the FC's own receiver is ignored, and the throttle
 *          below the FC's failsafe threshold, so the FC holds its RC failsafe
 *          until the pilot is back.
 * - local: an operator at the station took it back from the pilot, whose
 *          session is not closed yet. Only the local endpoint passes (and the
 *          always-pass components); with no local endpoint bound, everything
 *          but the pilot's endpoint.
 *
 * Above all three sits the priority component, if one is configured (the
 * browser's virtual sticks, mavlink_client's set_sbus): while it holds the
 * sticks -- it sent a frame with a value in it within the priority timeout --
 * it passes in every mode and every other RC source is dropped, the pilot's
 * included, so there is one source of truth for them rather than two frames
 * taking turns. A frame of it that holds nothing (it hands its channels back)
 * ends the hold at once. In pilot mode it counts as the pilot flying: the
 * pilot's virtual sticks reach the station as this component, and no failsafe
 * is sent while it drives.
 *
 * The mode is latched: the controller going quiet changes nothing, so its
 * crash never hands the sticks to somebody else. The router starts open.
 *
 * The priority and always-pass components share an endpoint (mavlink_client's
 * sends both), so those two are still told apart by compid; the priority
 * component can be bound to the endpoint its frames must arrive on, and the
 * pilot also to the address they must come from (the controller names it with
 * the mode: "pilot <ip>"). A frame on the pilot's or the local endpoint is
 * that transmitter's whatever compid it carries -- never the priority
 * component, never an always-pass one -- and a priority compid from anywhere
 * else is an ordinary source: dropped in pilot and local mode, never holding
 * the sticks over anyone.
 *
 * The always-pass components can be made to need a stick source: any RC frame
 * reaching the FC tells it the radio is alive, so a frame driving only the
 * digital link's channels, re-sent every half second, would keep the FC from
 * ever noticing the transmitter is gone -- no RC failsafe, the last sticks held.
 * With a pass timeout they pass only while some stick source (the pilot, a
 * local transmitter, the virtual sticks) has passed within it -- the router's
 * own failsafe frame does not count. Until the first stick source they pass
 * anyway: a flight with no transmitter at all still switches its video link.
 */
class RcGate {
public:
    enum class Mode { Open, Pilot, Local };

    // A failsafe template entry that repeats the pilot's last value; anything
    // else is sent as it is (UINT16_MAX: leave the channel as it is).
    static constexpr int32_t LAST = -1;
    static constexpr size_t CHANNELS = 18;
    using Template = std::array<int32_t, CHANNELS>;

    RcGate() = default;

    /*
     * @pass_compids: components that pass in every mode.
     * @failsafe_timeout_ms: pilot silence before the failsafe frame; 0 never.
     * @failsafe: the failsafe frame, see parse_failsafe().
     */
    void configure(std::vector<uint8_t> pass_compids, uint32_t failsafe_timeout_ms,
                   const Template &failsafe);

    /*
     * @compid: the priority component, see above.
     * @timeout_ms: how long one of its frames holds the sticks.
     */
    void set_priority(uint8_t compid, uint32_t timeout_ms);

    /*
     * Endpoint names. @pilot_endpoint: the remote pilot's transmitter; empty
     * is no pilot at all. @local_endpoint: this station's own; empty lets
     * everyone but the pilot drive in local mode. @priority_endpoint: where
     * the priority component's frames must arrive; empty accepts any.
     */
    void bind_endpoints(std::string pilot_endpoint, std::string local_endpoint,
                        std::string priority_endpoint);

    /*
     * @timeout_ms: how long after the last stick frame the always-pass
     * components still pass; 0 lets them pass always.
     */
    void set_pass_timeout(uint32_t timeout_ms) { _pass_timeout_ms = timeout_ms; }

    /*
     * Whether the always-pass components went from passing to held back, or
     * back, since the last call; @blocked says which. For the log.
     */
    bool take_pass_change(bool *blocked);

    static bool is_rc(uint32_t msg_id);

    /*
     * Rewrites the source sysid of one whole MAVLink frame (v1 or v2, as it
     * came off the wire) and its checksum to match. Returns false and leaves
     * the frame alone when it is signed (a new sysid would break the
     * signature), truncated or of an unknown message.
     *
     * For RcPilotSysid, a transitional measure: pilots up to 3.6.12 send
     * their RC as crsf2mav's default sysid 254, which the FC drops (it takes
     * overrides from SYSID_MYGCS only); from 3.7.0 they send 255 themselves.
     * TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
     */
    static bool patch_sysid(uint8_t *frame, unsigned int len, uint8_t sysid);

    // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
    const std::string &pilot_endpoint() const { return _pilot_endpoint; }

    bool enabled() const { return _enabled; }
    uint32_t failsafe_timeout_ms() const { return _timeout_ms; }

    /*
     * "open", "pilot" or "local", surrounding whitespace ignored. "pilot" may
     * be followed by the pilot's address ("pilot 100.64.1.2"); @pilot_ip is
     * set to it, or emptied when there is none.
     */
    static bool parse_mode(const std::string &text, Mode *mode, std::string *pilot_ip);
    static const char *mode_name(Mode mode);

    /*
     * Comma-separated, channel 1 first: a pulse width 0..65535, "last" or
     * "ignore" (65535). Channels not listed are "last". Empty is all "last".
     */
    static bool parse_failsafe(const std::string &text, Template *out);

    /*
     * Returns true when the mode changed. Entering pilot mode gives the pilot
     * the whole timeout before the first failsafe frame. @pilot_ip: the address
     * the pilot's frames must come from in pilot mode, empty for any; a new one
     * alone is not a change of mode.
     */
    bool set_mode(Mode mode, usec_t now, const std::string &pilot_ip = "");
    Mode mode() const { return _mode; }
    const std::string &pilot_ip() const { return _pilot_ip; }

    /*
     * Returns false if the message must be dropped. Records the pilot's
     * frames, in every mode, so "last" has a value from the first one on.
     */
    bool check(uint32_t msg_id, uint8_t src_compid, const uint8_t *payload, uint8_t payload_len,
               usec_t now, const std::string &endpoint = "", const std::string &peer = "");

    /*
     * Whether the failsafe frame is due: pilot mode and the pilot silent for
     * the timeout.
     */
    bool failsafe_due(usec_t now) const;

    /*
     * The failsafe frame's 18 channels.
     */
    void failsafe_channels(uint16_t out[CHANNELS]) const;

    /*
     * Hands back to the RC radio the channels the last frame that held the
     * sticks overrode -- the failsafe frame's if @after_failsafe, else the
     * pilot's -- and leaves every other one as it is, the always-pass ones'
     * (the digital link's) above all. Release is 0 on 1-8 and UINT16_MAX-1 on
     * 9-18, where 0 means "leave it as it is". Returns false when there is
     * nothing to release.
     */
    bool release_channels(bool after_failsafe, uint16_t out[CHANNELS]) const;

private:
    enum class Source { Pilot, Local, Priority, Pass, Other };

    Source _source(uint8_t src_compid, const std::string &endpoint) const;
    bool _check_sticks(Source source, uint32_t msg_id, const uint8_t *payload,
                       uint8_t payload_len, usec_t now, const std::string &peer);
    bool _pass_allowed(usec_t now) const;

    bool _enabled = false;
    std::vector<uint8_t> _pass;
    uint32_t _timeout_ms = 0;
    Template _failsafe{};

    bool _has_priority = false;
    uint8_t _priority = 0;
    uint32_t _priority_timeout_ms = 0;
    usec_t _priority_seen = 0; // 0: not holding
    std::string _pilot_endpoint;
    std::string _local_endpoint;
    std::string _priority_endpoint;
    std::string _pilot_ip;
    uint32_t _pass_timeout_ms = 0;
    usec_t _stick_seen = 0; // last stick frame passed; 0: none yet
    bool _pass_blocked = false;
    bool _pass_changed = false;

    Mode _mode = Mode::Open;
    usec_t _pilot_seen = 0; // last pilot frame, or entering pilot mode
    std::array<uint16_t, CHANNELS> _pilot_last{};
    bool _have_last = false;
};
