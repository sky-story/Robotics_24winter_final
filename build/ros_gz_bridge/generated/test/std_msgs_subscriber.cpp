// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <gtest/gtest.h>

#include <chrono>

#include "ros_subscriber.hpp"

using ros_subscriber::MyTestClass;

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, boolean_bool)
{
  MyTestClass<std_msgs::msg::Bool> client("boolean_bool");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, color_colorrgba)
{
  MyTestClass<std_msgs::msg::ColorRGBA> client("color_colorrgba");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, empty_empty)
{
  MyTestClass<std_msgs::msg::Empty> client("empty_empty");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, float_float32)
{
  MyTestClass<std_msgs::msg::Float32> client("float_float32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, double_float64)
{
  MyTestClass<std_msgs::msg::Float64> client("double_float64");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, header_header)
{
  MyTestClass<std_msgs::msg::Header> client("header_header");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, int32_int32)
{
  MyTestClass<std_msgs::msg::Int32> client("int32_int32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, stringmsg_string)
{
  MyTestClass<std_msgs::msg::String> client("stringmsg_string");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, uint32_uint32)
{
  MyTestClass<std_msgs::msg::UInt32> client("uint32_uint32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

