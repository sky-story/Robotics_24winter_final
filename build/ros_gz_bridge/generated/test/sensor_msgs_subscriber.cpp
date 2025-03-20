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
TEST(ROSSubscriberTest, batterystate_batterystate)
{
  MyTestClass<sensor_msgs::msg::BatteryState> client("batterystate_batterystate");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, camerainfo_camerainfo)
{
  MyTestClass<sensor_msgs::msg::CameraInfo> client("camerainfo_camerainfo");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, fluidpressure_fluidpressure)
{
  MyTestClass<sensor_msgs::msg::FluidPressure> client("fluidpressure_fluidpressure");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, image_image)
{
  MyTestClass<sensor_msgs::msg::Image> client("image_image");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, imu_imu)
{
  MyTestClass<sensor_msgs::msg::Imu> client("imu_imu");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, model_jointstate)
{
  MyTestClass<sensor_msgs::msg::JointState> client("model_jointstate");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, joy_joy)
{
  MyTestClass<sensor_msgs::msg::Joy> client("joy_joy");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, laserscan_laserscan)
{
  MyTestClass<sensor_msgs::msg::LaserScan> client("laserscan_laserscan");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, magnetometer_magneticfield)
{
  MyTestClass<sensor_msgs::msg::MagneticField> client("magnetometer_magneticfield");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, navsat_navsatfix)
{
  MyTestClass<sensor_msgs::msg::NavSatFix> client("navsat_navsatfix");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pointcloudpacked_pointcloud2)
{
  MyTestClass<sensor_msgs::msg::PointCloud2> client("pointcloudpacked_pointcloud2");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

