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

#include "message_disable.h"

#include <gtest/gtest.h>

TEST(MessageDisableTest, EmptyIsDisabled)
{
    MessageDisable d;
    EXPECT_FALSE(d.enabled());
    d.configure({});
    EXPECT_FALSE(d.enabled());
}

static const uintptr_t LINK_A = 1, LINK_B = 2;
static const uint16_t FC = (1 << 8) | 1;
static const usec_t T0 = 1000 * USEC_PER_SEC;

TEST(MessageDisableTest, UnlistedIsNeverSent)
{
    MessageDisable d;
    d.configure({87, 62});
    EXPECT_TRUE(d.listed(62));
    EXPECT_FALSE(d.listed(33));
    EXPECT_FALSE(d.heard(LINK_A, FC, 33, T0));
    EXPECT_FALSE(d.heard(LINK_A, FC, 0, T0));
}

TEST(MessageDisableTest, SentOnFirstSight)
{
    MessageDisable d;
    d.configure({87, 62});
    EXPECT_TRUE(d.heard(LINK_A, FC, 87, T0));
}

// Copies sent before the autopilot obeyed keep arriving: one command for them.
TEST(MessageDisableTest, OneCommandPerMessagePerHoldoff)
{
    MessageDisable d;
    d.configure({87});
    ASSERT_TRUE(d.heard(LINK_A, FC, 87, T0));
    EXPECT_FALSE(d.heard(LINK_A, FC, 87, T0 + USEC_PER_SEC));
    EXPECT_FALSE(d.heard(LINK_A, FC, 87, T0 + MessageDisable::HOLDOFF - 1));
    // still coming: the command was lost, send it again
    EXPECT_TRUE(d.heard(LINK_A, FC, 87, T0 + MessageDisable::HOLDOFF));
}

// After a reboot every listed message is back at once: no burst in the uplink.
TEST(MessageDisableTest, OneCommandPerLinkPerSpacing)
{
    MessageDisable d;
    d.configure({87, 62, 11039});
    ASSERT_TRUE(d.heard(LINK_A, FC, 87, T0));
    EXPECT_FALSE(d.heard(LINK_A, FC, 62, T0 + 1));
    EXPECT_FALSE(d.heard(LINK_A, FC, 11039, T0 + MessageDisable::SPACING - 1));
    // a refused message gets its turn with a later copy
    EXPECT_TRUE(d.heard(LINK_A, FC, 62, T0 + MessageDisable::SPACING));
    EXPECT_FALSE(d.heard(LINK_A, FC, 11039, T0 + MessageDisable::SPACING + 1));
    EXPECT_TRUE(d.heard(LINK_A, FC, 11039, T0 + 2 * MessageDisable::SPACING));
}

// The autopilot keeps the setting per link: each link is commanded on its own.
TEST(MessageDisableTest, LinksAreIndependent)
{
    MessageDisable d;
    d.configure({87});
    ASSERT_TRUE(d.heard(LINK_A, FC, 87, T0));
    EXPECT_TRUE(d.heard(LINK_B, FC, 87, T0));
}

TEST(MessageDisableTest, AutopilotsShareTheLinksSpacing)
{
    MessageDisable d;
    d.configure({87});
    const uint16_t other = (2 << 8) | 1;
    ASSERT_TRUE(d.heard(LINK_A, FC, 87, T0));
    EXPECT_FALSE(d.heard(LINK_A, other, 87, T0 + 1));
    EXPECT_TRUE(d.heard(LINK_A, other, 87, T0 + MessageDisable::SPACING));
}

TEST(MessageDisableTest, ReconfigureForgets)
{
    MessageDisable d;
    d.configure({87});
    ASSERT_TRUE(d.heard(LINK_A, FC, 87, T0));
    d.configure({87, 62});
    EXPECT_TRUE(d.heard(LINK_A, FC, 87, T0 + 1));
}

TEST(MessageDisableTest, PacksSetMessageIntervalMinusOne)
{
    mavlink_message_t msg;
    MessageDisable::pack(255, 230, (1 << 8) | 1, 11039, &msg);

    EXPECT_EQ(msg.msgid, (uint32_t)MAVLINK_MSG_ID_COMMAND_LONG);
    EXPECT_EQ(msg.sysid, 255);
    EXPECT_EQ(msg.compid, 230);

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    EXPECT_EQ(cmd.target_system, 1);
    EXPECT_EQ(cmd.target_component, 1);
    EXPECT_EQ(cmd.command, MAV_CMD_SET_MESSAGE_INTERVAL);
    EXPECT_EQ(cmd.confirmation, 0);
    EXPECT_EQ(cmd.param1, 11039.0f); // a 24-bit id is exact in a float
    EXPECT_EQ(cmd.param2, -1.0f);    // -1: stop sending
}
