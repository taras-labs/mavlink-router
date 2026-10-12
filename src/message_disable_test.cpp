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

TEST(MessageDisableTest, OneMessagePerTickRoundRobin)
{
    MessageDisable d;
    d.configure({87, 62, 11039});
    ASSERT_TRUE(d.enabled());
    EXPECT_EQ(d.next(), 87u);
    EXPECT_EQ(d.next(), 62u);
    EXPECT_EQ(d.next(), 11039u);
    EXPECT_EQ(d.next(), 87u); // and again, for ever
}

TEST(MessageDisableTest, ReconfigureStartsOver)
{
    MessageDisable d;
    d.configure({1, 2, 3});
    d.next();
    d.configure({7, 8});
    EXPECT_EQ(d.next(), 7u);
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
