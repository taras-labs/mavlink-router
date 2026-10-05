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

#include <gtest/gtest.h>

static const uint32_t RC_OVERRIDE = 70;
static const uint32_t TIMESYNC = 111;
static const uint8_t REMOTE = 190;
static const uint8_t LOCAL = 191;
static const int FC = 1;

static usec_t ms(unsigned v)
{
    // never 0: 0 is "never seen" inside CompPriority
    return (usec_t)(v + 1000) * USEC_PER_MSEC;
}

static CompPriority make()
{
    CompPriority p;
    p.configure({REMOTE, LOCAL}, {RC_OVERRIDE}, 500);
    return p;
}

TEST(CompPriorityTest, DisabledPassesEverything)
{
    CompPriority p;
    EXPECT_FALSE(p.enabled());
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(1)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(2)));
}

TEST(CompPriorityTest, LoneLowerPriorityPasses)
{
    auto p = make();
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(40)));
}

TEST(CompPriorityTest, HigherPriorityTakesOverOnFirstFrame)
{
    auto p = make();
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(10)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(20)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(30)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(40)));
}

TEST(CompPriorityTest, FallsBackOnlyAfterTimeout)
{
    auto p = make();
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(0)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(250)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(500))); // exactly the timeout
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(501)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, LOCAL, ms(900)));
    // and back again the moment the higher one returns
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(910)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(920)));
}

TEST(CompPriorityTest, UnlistedComponentsAndMessagesUntouched)
{
    auto p = make();
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, REMOTE, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, 0, ms(10)));     // e.g. a GCS at compid 0
    EXPECT_TRUE(p.check(TIMESYNC, FC, LOCAL, ms(20)));    // not arbitrated
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, LOCAL, ms(30)));
}

TEST(CompPriorityTest, TargetsArbitratedSeparately)
{
    auto p = make();
    EXPECT_TRUE(p.check(RC_OVERRIDE, 1, REMOTE, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, 2, LOCAL, ms(10)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, 1, LOCAL, ms(20)));
}

TEST(CompPriorityTest, ThreeLevels)
{
    CompPriority p;
    p.configure({10, 20, 30}, {RC_OVERRIDE}, 100);
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, 30, ms(0)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, 20, ms(10)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, 30, ms(20)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, 10, ms(30)));
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, 20, ms(40)));
    // 10 goes quiet; 20 was seen at 40 so it is next in line, not 30
    EXPECT_FALSE(p.check(RC_OVERRIDE, FC, 30, ms(135)));
    EXPECT_TRUE(p.check(RC_OVERRIDE, FC, 20, ms(136)));
}
