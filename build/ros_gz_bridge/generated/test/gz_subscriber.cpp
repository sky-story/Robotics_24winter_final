// Copyright 2022 Open Source Robotics Foundation, Inc.
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


#include <gtest/gtest.h>
#include <gz/transport.hh>

#include <chrono>
#include <memory>
#include <string>

#include "utils/test_utils.hpp"
#include "utils/gz_test_msg.hpp"

//////////////////////////////////////////////////
/// \brief A class for testing Gazebo Transport topic subscription.
template<typename GZ_T>
class MyTestClass
{
/// \brief Class constructor.
/// \param[in] _topic Topic to subscribe.
public: explicit MyTestClass(const std::string & _topic)
{
  EXPECT_TRUE(this->node.Subscribe(_topic, &MyTestClass::Cb, this));
}

/// \brief Member function called each time a topic update is received.
/// \param[in] _msg Gazebo message to be validated
public: void Cb(const GZ_T & _msg)
{
  ros_gz_bridge::testing::compareTestMsg(std::make_shared<GZ_T>(_msg));
  this->callbackExecuted = true;
}

/// \brief Member variables that flag when the actions are executed.
public: bool callbackExecuted = false;

/// \brief Transport node;
private: gz::transport::Node node;
};

/////////////////////////////////////////////////
TEST(GzSubscriberTest, actuators_actuators)
{
  MyTestClass<gz::msgs::Actuators> client("actuators_actuators");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, time_time)
{
  MyTestClass<gz::msgs::Time> client("time_time");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, vector3d_point)
{
  MyTestClass<gz::msgs::Vector3d> client("vector3d_point");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_pose)
{
  MyTestClass<gz::msgs::Pose> client("pose_pose");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_v_posearray)
{
  MyTestClass<gz::msgs::Pose_V> client("pose_v_posearray");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_posestamped)
{
  MyTestClass<gz::msgs::Pose> client("pose_posestamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, posewithcovariance_posewithcovariance)
{
  MyTestClass<gz::msgs::PoseWithCovariance> client("posewithcovariance_posewithcovariance");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, quaternion_quaternion)
{
  MyTestClass<gz::msgs::Quaternion> client("quaternion_quaternion");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_transform)
{
  MyTestClass<gz::msgs::Pose> client("pose_transform");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_transformstamped)
{
  MyTestClass<gz::msgs::Pose> client("pose_transformstamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, twist_twist)
{
  MyTestClass<gz::msgs::Twist> client("twist_twist");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, twist_twiststamped)
{
  MyTestClass<gz::msgs::Twist> client("twist_twiststamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, twistwithcovariance_twistwithcovariance)
{
  MyTestClass<gz::msgs::TwistWithCovariance> client("twistwithcovariance_twistwithcovariance");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, twistwithcovariance_twistwithcovariancestamped)
{
  MyTestClass<gz::msgs::TwistWithCovariance> client("twistwithcovariance_twistwithcovariancestamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, vector3d_vector3)
{
  MyTestClass<gz::msgs::Vector3d> client("vector3d_vector3");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, wrench_wrench)
{
  MyTestClass<gz::msgs::Wrench> client("wrench_wrench");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, wrench_wrenchstamped)
{
  MyTestClass<gz::msgs::Wrench> client("wrench_wrenchstamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, navsat_gpsfix)
{
  MyTestClass<gz::msgs::NavSat> client("navsat_gpsfix");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, odometry_odometry)
{
  MyTestClass<gz::msgs::Odometry> client("odometry_odometry");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, odometrywithcovariance_odometry)
{
  MyTestClass<gz::msgs::OdometryWithCovariance> client("odometrywithcovariance_odometry");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, any_parametervalue)
{
  MyTestClass<gz::msgs::Any> client("any_parametervalue");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, altimeter_altimeter)
{
  MyTestClass<gz::msgs::Altimeter> client("altimeter_altimeter");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, contact_contact)
{
  MyTestClass<gz::msgs::Contact> client("contact_contact");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, contacts_contacts)
{
  MyTestClass<gz::msgs::Contacts> client("contacts_contacts");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, dataframe_dataframe)
{
  MyTestClass<gz::msgs::Dataframe> client("dataframe_dataframe");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, entity_entity)
{
  MyTestClass<gz::msgs::Entity> client("entity_entity");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, entitywrench_entitywrench)
{
  MyTestClass<gz::msgs::EntityWrench> client("entitywrench_entitywrench");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, float_v_float32array)
{
  MyTestClass<gz::msgs::Float_V> client("float_v_float32array");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, guicamera_guicamera)
{
  MyTestClass<gz::msgs::GUICamera> client("guicamera_guicamera");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, jointwrench_jointwrench)
{
  MyTestClass<gz::msgs::JointWrench> client("jointwrench_jointwrench");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, light_light)
{
  MyTestClass<gz::msgs::Light> client("light_light");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, param_paramvec)
{
  MyTestClass<gz::msgs::Param> client("param_paramvec");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, param_v_paramvec)
{
  MyTestClass<gz::msgs::Param_V> client("param_v_paramvec");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, sensornoise_sensornoise)
{
  MyTestClass<gz::msgs::SensorNoise> client("sensornoise_sensornoise");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, stringmsg_v_stringvec)
{
  MyTestClass<gz::msgs::StringMsg_V> client("stringmsg_v_stringvec");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, trackvisual_trackvisual)
{
  MyTestClass<gz::msgs::TrackVisual> client("trackvisual_trackvisual");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, videorecord_videorecord)
{
  MyTestClass<gz::msgs::VideoRecord> client("videorecord_videorecord");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, clock_clock)
{
  MyTestClass<gz::msgs::Clock> client("clock_clock");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, batterystate_batterystate)
{
  MyTestClass<gz::msgs::BatteryState> client("batterystate_batterystate");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, camerainfo_camerainfo)
{
  MyTestClass<gz::msgs::CameraInfo> client("camerainfo_camerainfo");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, fluidpressure_fluidpressure)
{
  MyTestClass<gz::msgs::FluidPressure> client("fluidpressure_fluidpressure");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, image_image)
{
  MyTestClass<gz::msgs::Image> client("image_image");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, imu_imu)
{
  MyTestClass<gz::msgs::IMU> client("imu_imu");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, model_jointstate)
{
  MyTestClass<gz::msgs::Model> client("model_jointstate");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, joy_joy)
{
  MyTestClass<gz::msgs::Joy> client("joy_joy");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, laserscan_laserscan)
{
  MyTestClass<gz::msgs::LaserScan> client("laserscan_laserscan");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, magnetometer_magneticfield)
{
  MyTestClass<gz::msgs::Magnetometer> client("magnetometer_magneticfield");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, navsat_navsatfix)
{
  MyTestClass<gz::msgs::NavSat> client("navsat_navsatfix");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pointcloudpacked_pointcloud2)
{
  MyTestClass<gz::msgs::PointCloudPacked> client("pointcloudpacked_pointcloud2");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, boolean_bool)
{
  MyTestClass<gz::msgs::Boolean> client("boolean_bool");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, color_colorrgba)
{
  MyTestClass<gz::msgs::Color> client("color_colorrgba");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, empty_empty)
{
  MyTestClass<gz::msgs::Empty> client("empty_empty");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, float_float32)
{
  MyTestClass<gz::msgs::Float> client("float_float32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, double_float64)
{
  MyTestClass<gz::msgs::Double> client("double_float64");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, header_header)
{
  MyTestClass<gz::msgs::Header> client("header_header");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, int32_int32)
{
  MyTestClass<gz::msgs::Int32> client("int32_int32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, stringmsg_string)
{
  MyTestClass<gz::msgs::StringMsg> client("stringmsg_string");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, uint32_uint32)
{
  MyTestClass<gz::msgs::UInt32> client("uint32_uint32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, pose_v_tfmessage)
{
  MyTestClass<gz::msgs::Pose_V> client("pose_v_tfmessage");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, jointtrajectory_jointtrajectory)
{
  MyTestClass<gz::msgs::JointTrajectory> client("jointtrajectory_jointtrajectory");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, annotatedaxisaligned2dbox_detection2d)
{
  MyTestClass<gz::msgs::AnnotatedAxisAligned2DBox> client("annotatedaxisaligned2dbox_detection2d");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, annotatedaxisaligned2dbox_v_detection2darray)
{
  MyTestClass<gz::msgs::AnnotatedAxisAligned2DBox_V> client("annotatedaxisaligned2dbox_v_detection2darray");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, annotatedoriented3dbox_detection3d)
{
  MyTestClass<gz::msgs::AnnotatedOriented3DBox> client("annotatedoriented3dbox_detection3d");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, annotatedoriented3dbox_v_detection3darray)
{
  MyTestClass<gz::msgs::AnnotatedOriented3DBox_V> client("annotatedoriented3dbox_v_detection3darray");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
