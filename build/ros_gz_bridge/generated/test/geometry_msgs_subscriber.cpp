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
TEST(ROSSubscriberTest, vector3d_point)
{
  MyTestClass<geometry_msgs::msg::Point> client("vector3d_point");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pose_pose)
{
  MyTestClass<geometry_msgs::msg::Pose> client("pose_pose");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pose_v_posearray)
{
  MyTestClass<geometry_msgs::msg::PoseArray> client("pose_v_posearray");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pose_posestamped)
{
  MyTestClass<geometry_msgs::msg::PoseStamped> client("pose_posestamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, posewithcovariance_posewithcovariance)
{
  MyTestClass<geometry_msgs::msg::PoseWithCovariance> client("posewithcovariance_posewithcovariance");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, quaternion_quaternion)
{
  MyTestClass<geometry_msgs::msg::Quaternion> client("quaternion_quaternion");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pose_transform)
{
  MyTestClass<geometry_msgs::msg::Transform> client("pose_transform");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, pose_transformstamped)
{
  MyTestClass<geometry_msgs::msg::TransformStamped> client("pose_transformstamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, twist_twist)
{
  MyTestClass<geometry_msgs::msg::Twist> client("twist_twist");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, twist_twiststamped)
{
  MyTestClass<geometry_msgs::msg::TwistStamped> client("twist_twiststamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, twistwithcovariance_twistwithcovariance)
{
  MyTestClass<geometry_msgs::msg::TwistWithCovariance> client("twistwithcovariance_twistwithcovariance");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, twistwithcovariance_twistwithcovariancestamped)
{
  MyTestClass<geometry_msgs::msg::TwistWithCovarianceStamped> client("twistwithcovariance_twistwithcovariancestamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, vector3d_vector3)
{
  MyTestClass<geometry_msgs::msg::Vector3> client("vector3d_vector3");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, wrench_wrench)
{
  MyTestClass<geometry_msgs::msg::Wrench> client("wrench_wrench");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(ROSSubscriberTest, wrench_wrenchstamped)
{
  MyTestClass<geometry_msgs::msg::WrenchStamped> client("wrench_wrenchstamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVarAndSpin(
    ros_subscriber::TestNode(), client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

