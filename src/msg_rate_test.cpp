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

#include <gtest/gtest.h>

static const uintptr_t LINK_A = 1, LINK_B = 2;
static const usec_t T0 = 1000 * USEC_PER_SEC;
static const usec_t INTERVAL = 100 * USEC_PER_MSEC; // 10 Hz
static const uint32_t ATTITUDE = 30;

static MsgRate::Verdict offer(MsgRate &r, uintptr_t link, usec_t now, uint8_t tag,
                              uint32_t msg_id = ATTITUDE, uint8_t sysid = 1)
{
    const uint8_t packet[2] = {tag, tag};
    return r.offer(link, msg_id, sysid, 1, INTERVAL, now, packet, sizeof(packet));
}

TEST(MsgRateTest, ParseRules)
{
    std::vector<MsgRateRule> rules;
    const char *list = "30:10, 33 : 2.5,24:0.5";
    ASSERT_EQ(0, MsgRate::parse(list, strlen(list), &rules, sizeof(rules)));
    ASSERT_EQ(3u, rules.size());
    EXPECT_EQ(30u, rules[0].msg_id);
    EXPECT_EQ(100000u, rules[0].interval);
    EXPECT_EQ(33u, rules[1].msg_id);
    EXPECT_EQ(400000u, rules[1].interval);
    EXPECT_EQ(2000000u, rules[2].interval);
}

TEST(MsgRateTest, ParseRefusesNonsense)
{
    for (const char *bad : {"30", "30:", ":10", "30:0", "30:-1", "30:abc", "x:10", "30:10,,33:5",
                            "30:10,", "30:5000", "-3:10"}) {
        std::vector<MsgRateRule> rules;
        EXPECT_NE(0, MsgRate::parse(bad, strlen(bad), &rules, sizeof(rules))) << bad;
        EXPECT_TRUE(rules.empty()) << bad; // nothing half-applied
    }
}

TEST(MsgRateTest, FirstGoesAtOnce)
{
    MsgRate r;
    EXPECT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    EXPECT_EQ(USEC_INFINITY, r.next_due());
}

TEST(MsgRateTest, SoonerIsHeldUntilTheIntervalIsOver)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    EXPECT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 30000, 2));
    EXPECT_EQ(T0 + INTERVAL, r.next_due());

    EXPECT_TRUE(r.take_due(T0 + INTERVAL - 1).empty());
    auto due = r.take_due(T0 + INTERVAL);
    ASSERT_EQ(1u, due.size());
    EXPECT_EQ(LINK_A, due[0].link);
    EXPECT_EQ(2, due[0].packet[0]);
    EXPECT_EQ(USEC_INFINITY, r.next_due());
    // nothing is sent twice
    EXPECT_TRUE(r.take_due(T0 + 10 * INTERVAL).empty());
}

TEST(MsgRateTest, FresherReplacesTheHeldOne)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 20000, 2));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 60000, 3));
    auto due = r.take_due(T0 + INTERVAL);
    ASSERT_EQ(1u, due.size());
    EXPECT_EQ(3, due[0].packet[0]); // only the fresh one
}

// The timer is late and a message arrives after the interval: it goes at once,
// and the held one, older, is never sent after it.
TEST(MsgRateTest, FreshAfterTheIntervalDropsTheHeldOne)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 50000, 2));
    EXPECT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0 + INTERVAL + 5000, 3));
    EXPECT_TRUE(r.take_due(T0 + 10 * INTERVAL).empty());
    EXPECT_EQ(USEC_INFINITY, r.next_due());
}

TEST(MsgRateTest, IntervalRunsFromTheLastSent)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 10000, 2));
    // released 8 ms late: the next one is measured from then
    ASSERT_EQ(1u, r.take_due(T0 + INTERVAL + 8000).size());
    EXPECT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + INTERVAL + 20000, 3));
    EXPECT_EQ(T0 + 2 * INTERVAL + 8000, r.next_due());
}

TEST(MsgRateTest, EveryEndpointMessageAndSenderIsItsOwn)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    EXPECT_EQ(MsgRate::Verdict::Send, offer(r, LINK_B, T0 + 1000, 1));          // other endpoint
    EXPECT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0 + 1000, 1, 33));      // other message
    EXPECT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0 + 1000, 1, ATTITUDE, 2)); // other vehicle
    EXPECT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 1000, 1));
}

TEST(MsgRateTest, ForgetDropsAClosedEndpoint)
{
    MsgRate r;
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_A, T0, 1));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_A, T0 + 1000, 2));
    ASSERT_EQ(MsgRate::Verdict::Send, offer(r, LINK_B, T0, 1));
    ASSERT_EQ(MsgRate::Verdict::Held, offer(r, LINK_B, T0 + 1000, 2));
    r.forget(LINK_A);
    auto due = r.take_due(T0 + INTERVAL);
    ASSERT_EQ(1u, due.size());
    EXPECT_EQ(LINK_B, due[0].link);
}
