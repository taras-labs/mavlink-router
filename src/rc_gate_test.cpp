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

#include <cstring>
#include <vector>

#include <common/mavlink.h>
#include <gtest/gtest.h>

static const uint32_t RC_OVERRIDE = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
static const uint32_t MANUAL = MAVLINK_MSG_ID_MANUAL_CONTROL;
static const uint32_t TIMESYNC = MAVLINK_MSG_ID_TIMESYNC;
static const uint8_t PILOT = 190;
static const uint8_t LOCAL_TX = 193;
static const uint8_t SBUS = 0;
static const uint8_t LINK = 194;
static const uint16_t IGNORE = UINT16_MAX;

// Where each source comes in, as the entrypoint names them: mavlink-client's
// endpoint (the virtual sticks and the digital link), the pilot's 14552, this
// station's 14554, and an endpoint that is none of them (a GCS on the TCP
// server, say).
static const char *LINK_EP = "out1";
static const char *PILOT_EP = "out2";
static const char *LOCAL_EP = "out3";
static const char *OTHER_EP = "tcp";

static usec_t ms(unsigned v)
{
    return (usec_t)(v + 1000) * USEC_PER_MSEC;
}

static RcGate make(uint32_t timeout_ms = 500, const char *failsafe = "1500,1500,900,1500")
{
    RcGate g;
    RcGate::Template t;
    EXPECT_TRUE(RcGate::parse_failsafe(failsafe, &t));
    g.configure({LINK}, timeout_ms, t);
    g.bind_endpoints(PILOT_EP, LOCAL_EP, LINK_EP);
    return g;
}

// A pilot frame as crsf2mav sends it: sticks on 1-12, 6 and 7 left alone,
// nothing on 13-18. MAVLink 2 trims the trailing zeros.
struct Frame {
    uint8_t payload[sizeof(mavlink_rc_channels_override_t)];
    uint8_t len;
};

static Frame pilot_frame(uint16_t throttle)
{
    mavlink_rc_channels_override_t ov = {};
    ov.target_system = 1;
    ov.target_component = 1;
    uint16_t ch[12] = {1510, 1490, throttle, 1500, 1800, IGNORE, IGNORE, 1100, 1200, 1300, 1400, 1600};
    uint16_t *dst[12] = {&ov.chan1_raw, &ov.chan2_raw, &ov.chan3_raw, &ov.chan4_raw,
                         &ov.chan5_raw, &ov.chan6_raw, &ov.chan7_raw, &ov.chan8_raw,
                         &ov.chan9_raw, &ov.chan10_raw, &ov.chan11_raw, &ov.chan12_raw};
    for (int i = 0; i < 12; i++) {
        *dst[i] = ch[i];
    }
    Frame f;
    memcpy(f.payload, &ov, sizeof(ov));
    f.len = sizeof(ov);
    while (f.len > 1 && f.payload[f.len - 1] == 0) {
        f.len--;
    }
    return f;
}

static RcGate make_with_sbus(uint32_t priority_ms = 1500)
{
    auto g = make();
    g.set_priority(SBUS, priority_ms);
    return g;
}

// A frame from the virtual sticks: a value on 1-4, the rest left alone; or one
// handing everything back (0 on 1-8, "leave" elsewhere).
static bool send_sbus(RcGate &g, unsigned at, bool release = false)
{
    mavlink_rc_channels_override_t ov = {};
    uint16_t v = release ? 0 : 1500;
    ov.chan1_raw = ov.chan2_raw = ov.chan3_raw = ov.chan4_raw = v;
    ov.chan5_raw = ov.chan6_raw = ov.chan7_raw = ov.chan8_raw = release ? 0 : IGNORE;
    ov.chan9_raw = ov.chan10_raw = IGNORE;
    return g.check(RC_OVERRIDE, SBUS, (const uint8_t *)&ov, sizeof(ov), ms(at), LINK_EP);
}

// A frame from @compid as it arrives on its usual endpoint.
static const char *home_of(uint8_t compid)
{
    switch (compid) {
    case PILOT:
        return PILOT_EP;
    case LOCAL_TX:
        return LOCAL_EP;
    case SBUS:
    case LINK:
        return LINK_EP;
    }
    return OTHER_EP;
}

static bool send(RcGate &g, uint32_t msg, uint8_t compid, unsigned at)
{
    return g.check(msg, compid, nullptr, 0, ms(at), home_of(compid));
}

static bool send_pilot(RcGate &g, unsigned at, uint16_t throttle = 1300)
{
    Frame f = pilot_frame(throttle);
    return g.check(RC_OVERRIDE, PILOT, f.payload, f.len, ms(at), PILOT_EP);
}

TEST(RcGateTest, UnconfiguredPassesEverything)
{
    RcGate g;
    EXPECT_FALSE(g.enabled());
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 0));
    EXPECT_FALSE(g.failsafe_due(ms(10000)));
}

TEST(RcGateTest, OpenPassesEveryone)
{
    auto g = make();
    EXPECT_EQ(g.mode(), RcGate::Mode::Open);
    EXPECT_TRUE(send_pilot(g, 0));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 10));
    EXPECT_TRUE(send(g, RC_OVERRIDE, SBUS, 20));
    EXPECT_TRUE(send(g, RC_OVERRIDE, 1, 30));
    EXPECT_FALSE(g.failsafe_due(ms(10000)));
}

TEST(RcGateTest, PilotModePassesOnlyThePilotAndTheAlwaysPassOnes)
{
    auto g = make();
    EXPECT_TRUE(g.set_mode(RcGate::Mode::Pilot, ms(0)));
    EXPECT_TRUE(send_pilot(g, 10));
    EXPECT_FALSE(send(g, RC_OVERRIDE, LOCAL_TX, 20));
    EXPECT_FALSE(send(g, MANUAL, LOCAL_TX, 25));
    EXPECT_FALSE(send(g, RC_OVERRIDE, SBUS, 30));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 40));
    EXPECT_TRUE(send(g, MANUAL, PILOT, 50));
}

TEST(RcGateTest, OtherMessagesAreNeverTouched)
{
    auto g = make();
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    EXPECT_TRUE(send(g, TIMESYNC, LOCAL_TX, 10));
    g.set_mode(RcGate::Mode::Local, ms(20));
    EXPECT_TRUE(send(g, TIMESYNC, PILOT, 30));
}

TEST(RcGateTest, LocalModePassesOnlyTheLocalTransmitter)
{
    auto g = make();
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    g.set_mode(RcGate::Mode::Local, ms(10));
    EXPECT_FALSE(send_pilot(g, 20));
    EXPECT_FALSE(send(g, MANUAL, PILOT, 25));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 30));
    EXPECT_TRUE(send(g, MANUAL, LOCAL_TX, 35));
    EXPECT_FALSE(send(g, RC_OVERRIDE, 1, 38)); // a stranger on another endpoint
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 50));
    EXPECT_FALSE(g.failsafe_due(ms(10000)));
}

TEST(RcGateTest, LocalModeWithNoLocalEndpointShutsOutOnlyThePilot)
{
    RcGate g;
    RcGate::Template t;
    ASSERT_TRUE(RcGate::parse_failsafe("", &t));
    g.configure({LINK}, 500, t);
    g.bind_endpoints(PILOT_EP, "", LINK_EP);
    g.set_mode(RcGate::Mode::Local, ms(0));
    EXPECT_FALSE(send_pilot(g, 10));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 20));
    EXPECT_TRUE(send(g, RC_OVERRIDE, 1, 30));
}

TEST(RcGateTest, EnteringPilotModeGivesThePilotTheWholeTimeout)
{
    auto g = make(500);
    g.set_mode(RcGate::Mode::Pilot, ms(1000));
    EXPECT_FALSE(g.failsafe_due(ms(1499)));
    EXPECT_TRUE(g.failsafe_due(ms(1500)));
}

TEST(RcGateTest, FailsafeWhileThePilotIsSilentAndNotWhileHeSends)
{
    auto g = make(500);
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    send_pilot(g, 100);
    EXPECT_FALSE(g.failsafe_due(ms(599)));
    EXPECT_TRUE(g.failsafe_due(ms(600)));
    // other sources do not keep the failsafe away
    send(g, RC_OVERRIDE, LOCAL_TX, 700);
    EXPECT_TRUE(g.failsafe_due(ms(700)));
    // the pilot's first frame does
    send_pilot(g, 800);
    EXPECT_FALSE(g.failsafe_due(ms(800)));
}

TEST(RcGateTest, ZeroTimeoutNeverSendsTheFailsafe)
{
    auto g = make(0);
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    EXPECT_FALSE(g.failsafe_due(ms(100000)));
}

TEST(RcGateTest, FailsafeFrameTakesLastFromThePilot)
{
    auto g = make(500, "1500,1500,900,1500");
    send_pilot(g, 0, 1700);
    uint16_t ch[RcGate::CHANNELS];
    g.failsafe_channels(ch);

    EXPECT_EQ(ch[0], 1500);
    EXPECT_EQ(ch[2], 900);      // throttle under the failsafe threshold
    EXPECT_EQ(ch[4], 1800);     // the pilot's mode switch stays where it was
    EXPECT_EQ(ch[5], IGNORE);   // the digital link's channels stay its own
    EXPECT_EQ(ch[6], IGNORE);
    EXPECT_EQ(ch[11], 1600);
    EXPECT_EQ(ch[12], 0);       // "leave it" on 13-18, as the pilot sent it
}

TEST(RcGateTest, FailsafeBeforeAnyPilotFrameLeavesTheRestAlone)
{
    auto g = make(500, "1500,1500,900,1500");
    uint16_t ch[RcGate::CHANNELS];
    g.failsafe_channels(ch);
    EXPECT_EQ(ch[2], 900);
    for (size_t i = 4; i < RcGate::CHANNELS; i++) {
        EXPECT_EQ(ch[i], IGNORE) << i;
    }
}

TEST(RcGateTest, ReleaseFreesOnlyWhatTheLastFrameHeld)
{
    auto g = make(500, "1500,1500,900,1500");
    uint16_t ch[RcGate::CHANNELS];
    EXPECT_FALSE(g.release_channels(false, ch)); // nothing held yet

    send_pilot(g, 0);
    ASSERT_TRUE(g.release_channels(false, ch));
    EXPECT_EQ(ch[0], 0);           // released, 1-8
    EXPECT_EQ(ch[5], IGNORE);      // never the digital link's
    EXPECT_EQ(ch[6], IGNORE);
    EXPECT_EQ(ch[8], IGNORE - 1);  // released, 9-18
    EXPECT_EQ(ch[12], IGNORE);     // the pilot never held 13-18

    RcGate fresh = make(500, "1500,1500,900,1500,ignore");
    ASSERT_TRUE(fresh.release_channels(true, ch));
    EXPECT_EQ(ch[2], 0);
    EXPECT_EQ(ch[4], IGNORE);
}

TEST(RcGateTest, ParsesModes)
{
    RcGate::Mode m;
    std::string ip = "stale";
    EXPECT_TRUE(RcGate::parse_mode("pilot\n", &m, &ip));
    EXPECT_EQ(m, RcGate::Mode::Pilot);
    EXPECT_EQ(ip, "");
    EXPECT_TRUE(RcGate::parse_mode("pilot 100.64.1.2\n", &m, &ip));
    EXPECT_EQ(ip, "100.64.1.2");
    EXPECT_TRUE(RcGate::parse_mode("  local ", &m, &ip));
    EXPECT_EQ(m, RcGate::Mode::Local);
    EXPECT_TRUE(RcGate::parse_mode("open", &m, &ip));
    EXPECT_EQ(m, RcGate::Mode::Open);
    EXPECT_FALSE(RcGate::parse_mode("PILOT", &m, &ip));
    EXPECT_FALSE(RcGate::parse_mode("", &m, &ip));
    EXPECT_FALSE(RcGate::parse_mode("pilot not-an-ip", &m, &ip));
    EXPECT_FALSE(RcGate::parse_mode("local 100.64.1.2", &m, &ip));
}

TEST(RcGateTest, ParsesTheFailsafeTemplate)
{
    RcGate::Template t;
    ASSERT_TRUE(RcGate::parse_failsafe("1500, 1500,900 ,1500,last,ignore", &t));
    EXPECT_EQ(t[2], 900);
    EXPECT_EQ(t[4], RcGate::LAST);
    EXPECT_EQ(t[5], (int32_t)IGNORE);
    EXPECT_EQ(t[17], RcGate::LAST);

    ASSERT_TRUE(RcGate::parse_failsafe("", &t));
    EXPECT_EQ(t[0], RcGate::LAST);

    EXPECT_FALSE(RcGate::parse_failsafe("1500,,900", &t));
    EXPECT_FALSE(RcGate::parse_failsafe("1500,abc", &t));
    EXPECT_FALSE(RcGate::parse_failsafe("70000", &t));
    EXPECT_FALSE(RcGate::parse_failsafe("-1", &t));
    EXPECT_FALSE(RcGate::parse_failsafe(
        "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19", &t));
}

TEST(RcGateTest, SettingTheSameModeChangesNothing)
{
    auto g = make(500);
    EXPECT_FALSE(g.set_mode(RcGate::Mode::Open, ms(0)));
    EXPECT_TRUE(g.set_mode(RcGate::Mode::Pilot, ms(0)));
    // a repeat does not restart the pilot's grace period
    EXPECT_FALSE(g.set_mode(RcGate::Mode::Pilot, ms(400)));
    EXPECT_TRUE(g.failsafe_due(ms(500)));
}

TEST(RcGateTest, PriorityHoldsEveryoneElseOffInEveryMode)
{
    for (auto mode : {RcGate::Mode::Open, RcGate::Mode::Pilot, RcGate::Mode::Local}) {
        auto g = make_with_sbus(1500);
        g.set_mode(mode, ms(0));
        EXPECT_TRUE(send_sbus(g, 10)) << RcGate::mode_name(mode);
        EXPECT_FALSE(send_pilot(g, 20)) << RcGate::mode_name(mode);
        EXPECT_FALSE(send(g, RC_OVERRIDE, LOCAL_TX, 30)) << RcGate::mode_name(mode);
        EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 40)) << RcGate::mode_name(mode);
    }
}

TEST(RcGateTest, PriorityHoldEndsAfterItsTimeout)
{
    auto g = make_with_sbus(1500);
    send_sbus(g, 0);
    EXPECT_FALSE(send(g, RC_OVERRIDE, LOCAL_TX, 1499));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 1500));
}

TEST(RcGateTest, PriorityReleaseFrameEndsTheHoldAtOnce)
{
    auto g = make_with_sbus(1500);
    send_sbus(g, 0);
    EXPECT_TRUE(send_sbus(g, 100, true)); // the release itself passes
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 110));
}

TEST(RcGateTest, PriorityInPilotModeKeepsTheFailsafeAway)
{
    auto g = make_with_sbus(1500);
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    for (unsigned t = 0; t <= 3000; t += 500) {
        send_sbus(g, t);
        EXPECT_FALSE(g.failsafe_due(ms(t + 100))) << t;
    }
    EXPECT_TRUE(g.failsafe_due(ms(3500)));
}

TEST(RcGateTest, NoPriorityConfiguredLeavesCompid0ToTheModes)
{
    auto g = make();
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    EXPECT_FALSE(send_sbus(g, 10));
    EXPECT_TRUE(send_pilot(g, 20));
}

// --- identities are the endpoints; a compid only names the sender

static const char *PILOT_IP = "100.64.1.2";

static RcGate make_bound()
{
    return make_with_sbus(1500);
}

static bool send_from(RcGate &g, uint8_t compid, const char *ep, const char *peer, unsigned at,
                      uint16_t value = 1500)
{
    mavlink_rc_channels_override_t ov = {};
    ov.chan1_raw = ov.chan2_raw = ov.chan3_raw = ov.chan4_raw = value;
    return g.check(RC_OVERRIDE, compid, (const uint8_t *)&ov, sizeof(ov), ms(at), ep, peer);
}

TEST(RcGateTest, PriorityCompidFromAnotherEndpointHoldsNothing)
{
    auto g = make_bound();
    // compid 0 written by somebody on the pilot's port
    EXPECT_TRUE(send_from(g, SBUS, PILOT_EP, "10.0.0.9", 0)); // open: an ordinary source
    EXPECT_TRUE(send_from(g, LOCAL_TX, LOCAL_EP, "127.0.0.1", 10)); // nobody is held off
    EXPECT_TRUE(send_from(g, SBUS, OTHER_EP, "10.0.0.9", 12));
    EXPECT_TRUE(send_from(g, LOCAL_TX, LOCAL_EP, "127.0.0.1", 14)); // still nobody
    g.set_mode(RcGate::Mode::Pilot, ms(20), PILOT_IP);
    EXPECT_FALSE(send_from(g, SBUS, PILOT_EP, "10.0.0.9", 30)); // pilot mode: dropped
    EXPECT_FALSE(send_from(g, SBUS, OTHER_EP, "10.0.0.9", 35));
    EXPECT_TRUE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 40));    // and the pilot not held off
    // the real one, from mavlink-client's endpoint, does hold
    EXPECT_TRUE(send_from(g, SBUS, LINK_EP, "172.18.0.5", 50));
    EXPECT_FALSE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 60));
}

TEST(RcGateTest, ThePilotMustComeFromItsEndpointAndAddress)
{
    auto g = make_bound();
    g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP);
    EXPECT_FALSE(send_from(g, PILOT, LOCAL_EP, PILOT_IP, 10));    // wrong port
    EXPECT_FALSE(send_from(g, PILOT, PILOT_EP, "10.0.0.9", 20));  // wrong sender
    EXPECT_TRUE(g.failsafe_due(ms(600)));                         // neither counts as the pilot
    EXPECT_TRUE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 700));
    EXPECT_FALSE(g.failsafe_due(ms(700)));
}

TEST(RcGateTest, NoAddressGivenAcceptsThePilotFromAnySender)
{
    auto g = make_bound();
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    EXPECT_TRUE(send_from(g, PILOT, PILOT_EP, "10.0.0.9", 10));
    EXPECT_FALSE(send_from(g, PILOT, OTHER_EP, "10.0.0.9", 20)); // the port still counts
}

TEST(RcGateTest, LocalModeShutsOutThePilotsPortWhoeverSends)
{
    auto g = make_bound();
    g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP);
    g.set_mode(RcGate::Mode::Local, ms(10));
    EXPECT_FALSE(send_from(g, PILOT, PILOT_EP, "10.0.0.9", 20));
    EXPECT_FALSE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 30));
}

TEST(RcGateTest, ANewAddressIsNotAChangeOfMode)
{
    auto g = make_bound();
    EXPECT_TRUE(g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP));
    EXPECT_FALSE(g.set_mode(RcGate::Mode::Pilot, ms(100), "100.64.1.3"));
    EXPECT_EQ(g.pilot_ip(), "100.64.1.3");
    EXPECT_FALSE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 200));
    EXPECT_TRUE(send_from(g, PILOT, PILOT_EP, "100.64.1.3", 210));
}

TEST(RcGateTest, AnyCompidOnThePilotsEndpointIsThePilot)
{
    auto g = make_bound();
    g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP);
    for (uint8_t compid : {PILOT, LOCAL_TX, (uint8_t)1, (uint8_t)250}) {
        EXPECT_TRUE(send_from(g, compid, PILOT_EP, PILOT_IP, 100)) << (int)compid;
    }
    EXPECT_FALSE(g.failsafe_due(ms(599)));
    // and its frame is the one the failsafe repeats (the frames above left
    // channel 5 at 0, this one puts the mode switch at 1800)
    Frame f = pilot_frame(1300);
    EXPECT_TRUE(g.check(RC_OVERRIDE, LOCAL_TX, f.payload, f.len, ms(700), PILOT_EP, PILOT_IP));
    uint16_t ch[RcGate::CHANNELS];
    g.failsafe_channels(ch);
    EXPECT_EQ(ch[4], 1800); // channel 5 is "last" in the default template
}

TEST(RcGateTest, ThePilotsCompidOnTheLocalEndpointIsTheLocalTransmitter)
{
    auto g = make_bound();
    g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP);
    EXPECT_FALSE(send_from(g, PILOT, LOCAL_EP, PILOT_IP, 10));
    EXPECT_TRUE(g.failsafe_due(ms(500))); // it kept nothing away
    g.set_mode(RcGate::Mode::Local, ms(600));
    EXPECT_TRUE(send_from(g, PILOT, LOCAL_EP, "127.0.0.1", 610));
    EXPECT_FALSE(send_from(g, LOCAL_TX, PILOT_EP, PILOT_IP, 620));
}

TEST(RcGateTest, PassAndPriorityCompidsOnATransmittersEndpointAreThatTransmitter)
{
    auto g = make_bound();
    g.set_pass_timeout(1500);
    g.set_mode(RcGate::Mode::Pilot, ms(0), PILOT_IP);
    // 194 written on the local port is the local transmitter: shut out
    EXPECT_FALSE(send_from(g, LINK, LOCAL_EP, "127.0.0.1", 10));
    // 0 written on it holds nothing over the pilot
    EXPECT_FALSE(send_from(g, SBUS, LOCAL_EP, "127.0.0.1", 20));
    EXPECT_TRUE(send_from(g, PILOT, PILOT_EP, PILOT_IP, 30));
    // and 194 on the pilot's port is the pilot flying
    EXPECT_TRUE(send_from(g, LINK, PILOT_EP, PILOT_IP, 400));
    EXPECT_FALSE(g.failsafe_due(ms(899)));
    // the real always-pass frame still passes on its compid
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 900));
}

// --- the always-pass frame needs a stick source

TEST(RcGateTest, PassFramePassesUntilTheFirstStickSource)
{
    auto g = make();
    g.set_pass_timeout(1500);
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 0));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 100000)); // a flight with no transmitter at all
}

TEST(RcGateTest, PassFrameIsHeldBackOnceTheSticksGoQuiet)
{
    auto g = make();
    g.set_pass_timeout(1500);
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 0));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 1499));
    bool blocked = true;
    EXPECT_FALSE(g.take_pass_change(&blocked));
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 1500)); // the FC must see the radio gone
    ASSERT_TRUE(g.take_pass_change(&blocked));
    EXPECT_TRUE(blocked);
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 60000)); // and keep seeing it
    EXPECT_TRUE(send(g, RC_OVERRIDE, LOCAL_TX, 60100));
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 60200));
    ASSERT_TRUE(g.take_pass_change(&blocked));
    EXPECT_FALSE(blocked);
}

TEST(RcGateTest, DroppedStickFramesAreNoStickSource)
{
    auto g = make(0); // no failsafe frame: only the stick frames decide
    g.set_pass_timeout(1500);
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    EXPECT_TRUE(send_pilot(g, 10));
    EXPECT_FALSE(send(g, RC_OVERRIDE, LOCAL_TX, 1000)); // shut out in pilot mode
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 1600));     // so it kept nothing alive
}

TEST(RcGateTest, TheRoutersOwnFailsafeIsNoStickSource)
{
    auto g = make(500);
    g.set_pass_timeout(300);
    g.set_mode(RcGate::Mode::Pilot, ms(0));
    send_pilot(g, 0);
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 400));  // pilot quiet
    EXPECT_TRUE(g.failsafe_due(ms(600)));
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 600));  // failsafe running: still held back
}

TEST(RcGateTest, VirtualSticksCountAsAStickSource)
{
    auto g = make_with_sbus(1500);
    g.set_pass_timeout(1500);
    send_sbus(g, 0);
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 1000));
    EXPECT_FALSE(send(g, RC_OVERRIDE, LINK, 1600));
}

TEST(RcGateTest, ZeroPassTimeoutAlwaysPasses)
{
    auto g = make();
    send(g, RC_OVERRIDE, LOCAL_TX, 0);
    EXPECT_TRUE(send(g, RC_OVERRIDE, LINK, 100000));
}

// --- RcPilotSysid's rewrite. TODO(pilot-sysid): remove once every pilot runs v3.7.0+.

// An RC override as a pilot's crsf2mav puts it on the wire: sysid 254.
static std::vector<uint8_t> wire_frame(bool mavlink1)
{
    mavlink_status_t *st = mavlink_get_channel_status(MAVLINK_COMM_0);
    if (mavlink1) {
        st->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    } else {
        st->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack_chan(254, PILOT, MAVLINK_COMM_0, &msg, 1, 1,
                                               1510, 1490, 1300, 1500, 1800, IGNORE, IGNORE,
                                               1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    uint8_t out[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(out, &msg);
    return std::vector<uint8_t>(out, out + n);
}

// Whether a receiver takes the frame, checksum included.
static bool parse(const std::vector<uint8_t> &frame, mavlink_message_t *msg)
{
    mavlink_status_t status = {};
    mavlink_get_channel_status(MAVLINK_COMM_1)->parse_state = MAVLINK_PARSE_STATE_IDLE;
    for (uint8_t b : frame) {
        if (mavlink_parse_char(MAVLINK_COMM_1, b, msg, &status)) {
            return true;
        }
    }
    return false;
}

TEST(RcGateTest, PatchSysidRewritesTheSenderAndTheChecksum)
{
    for (bool v1 : {false, true}) {
        auto frame = wire_frame(v1);
        ASSERT_EQ(frame[0], v1 ? MAVLINK_STX_MAVLINK1 : MAVLINK_STX);
        ASSERT_TRUE(RcGate::patch_sysid(frame.data(), frame.size(), 255)) << v1;

        mavlink_message_t msg;
        ASSERT_TRUE(parse(frame, &msg)) << v1; // the checksum still holds
        EXPECT_EQ(msg.sysid, 255) << v1;
        EXPECT_EQ(msg.compid, PILOT) << v1;
        mavlink_rc_channels_override_t ov;
        mavlink_msg_rc_channels_override_decode(&msg, &ov);
        EXPECT_EQ(ov.chan1_raw, 1510) << v1;
        EXPECT_EQ(ov.chan3_raw, 1300) << v1;
        EXPECT_EQ(ov.chan6_raw, IGNORE) << v1;
        EXPECT_EQ(ov.chan8_raw, 1100) << v1;
    }
}

TEST(RcGateTest, PatchSysidWithoutTheChecksumWouldBeDropped)
{
    // what the rewrite is for: a sysid changed alone fails every receiver
    auto frame = wire_frame(false);
    frame[5] = 255;
    mavlink_message_t msg;
    EXPECT_FALSE(parse(frame, &msg));
}

TEST(RcGateTest, PatchSysidLeavesSignedAndShortFramesAlone)
{
    auto signed_frame = wire_frame(false);
    signed_frame[2] |= MAVLINK_IFLAG_SIGNED;
    const auto before = signed_frame;
    EXPECT_FALSE(RcGate::patch_sysid(signed_frame.data(), signed_frame.size(), 255));
    EXPECT_EQ(signed_frame, before);

    auto short_frame = wire_frame(false);
    const auto short_before = short_frame;
    EXPECT_FALSE(RcGate::patch_sysid(short_frame.data(), short_frame.size() - 1, 255));
    EXPECT_EQ(short_frame, short_before);

    uint8_t junk[12] = {0x42};
    EXPECT_FALSE(RcGate::patch_sysid(junk, sizeof(junk), 255));
}
