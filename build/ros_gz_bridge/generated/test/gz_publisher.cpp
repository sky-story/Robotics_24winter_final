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

// This file is generated from test/resource/gz_publisher.cpp.em

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "utils/test_utils.hpp"
#include "utils/gz_test_msg.hpp"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
/// \param[in] _signal signal number (interrupt or terminate)
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM) {
    g_terminatePub = true;
  }
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  gz::transport::Node node;

  // gz.msgs.Actuators.
  auto actuators_actuators_pub =
    node.Advertise<gz::msgs::Actuators>("actuators_actuators");
  gz::msgs::Actuators actuators_actuators_msg;
  ros_gz_bridge::testing::createTestMsg(actuators_actuators_msg);

  // gz.msgs.Time.
  auto time_time_pub =
    node.Advertise<gz::msgs::Time>("time_time");
  gz::msgs::Time time_time_msg;
  ros_gz_bridge::testing::createTestMsg(time_time_msg);

  // gz.msgs.Vector3d.
  auto vector3d_point_pub =
    node.Advertise<gz::msgs::Vector3d>("vector3d_point");
  gz::msgs::Vector3d vector3d_point_msg;
  ros_gz_bridge::testing::createTestMsg(vector3d_point_msg);

  // gz.msgs.Pose.
  auto pose_pose_pub =
    node.Advertise<gz::msgs::Pose>("pose_pose");
  gz::msgs::Pose pose_pose_msg;
  ros_gz_bridge::testing::createTestMsg(pose_pose_msg);

  // gz.msgs.Pose_V.
  auto pose_v_posearray_pub =
    node.Advertise<gz::msgs::Pose_V>("pose_v_posearray");
  gz::msgs::Pose_V pose_v_posearray_msg;
  ros_gz_bridge::testing::createTestMsg(pose_v_posearray_msg);

  // gz.msgs.Pose.
  auto pose_posestamped_pub =
    node.Advertise<gz::msgs::Pose>("pose_posestamped");
  gz::msgs::Pose pose_posestamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_posestamped_msg);

  // gz.msgs.PoseWithCovariance.
  auto posewithcovariance_posewithcovariance_pub =
    node.Advertise<gz::msgs::PoseWithCovariance>("posewithcovariance_posewithcovariance");
  gz::msgs::PoseWithCovariance posewithcovariance_posewithcovariance_msg;
  ros_gz_bridge::testing::createTestMsg(posewithcovariance_posewithcovariance_msg);

  // gz.msgs.Quaternion.
  auto quaternion_quaternion_pub =
    node.Advertise<gz::msgs::Quaternion>("quaternion_quaternion");
  gz::msgs::Quaternion quaternion_quaternion_msg;
  ros_gz_bridge::testing::createTestMsg(quaternion_quaternion_msg);

  // gz.msgs.Pose.
  auto pose_transform_pub =
    node.Advertise<gz::msgs::Pose>("pose_transform");
  gz::msgs::Pose pose_transform_msg;
  ros_gz_bridge::testing::createTestMsg(pose_transform_msg);

  // gz.msgs.Pose.
  auto pose_transformstamped_pub =
    node.Advertise<gz::msgs::Pose>("pose_transformstamped");
  gz::msgs::Pose pose_transformstamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_transformstamped_msg);

  // gz.msgs.Twist.
  auto twist_twist_pub =
    node.Advertise<gz::msgs::Twist>("twist_twist");
  gz::msgs::Twist twist_twist_msg;
  ros_gz_bridge::testing::createTestMsg(twist_twist_msg);

  // gz.msgs.Twist.
  auto twist_twiststamped_pub =
    node.Advertise<gz::msgs::Twist>("twist_twiststamped");
  gz::msgs::Twist twist_twiststamped_msg;
  ros_gz_bridge::testing::createTestMsg(twist_twiststamped_msg);

  // gz.msgs.TwistWithCovariance.
  auto twistwithcovariance_twistwithcovariance_pub =
    node.Advertise<gz::msgs::TwistWithCovariance>("twistwithcovariance_twistwithcovariance");
  gz::msgs::TwistWithCovariance twistwithcovariance_twistwithcovariance_msg;
  ros_gz_bridge::testing::createTestMsg(twistwithcovariance_twistwithcovariance_msg);

  // gz.msgs.TwistWithCovariance.
  auto twistwithcovariance_twistwithcovariancestamped_pub =
    node.Advertise<gz::msgs::TwistWithCovariance>("twistwithcovariance_twistwithcovariancestamped");
  gz::msgs::TwistWithCovariance twistwithcovariance_twistwithcovariancestamped_msg;
  ros_gz_bridge::testing::createTestMsg(twistwithcovariance_twistwithcovariancestamped_msg);

  // gz.msgs.Vector3d.
  auto vector3d_vector3_pub =
    node.Advertise<gz::msgs::Vector3d>("vector3d_vector3");
  gz::msgs::Vector3d vector3d_vector3_msg;
  ros_gz_bridge::testing::createTestMsg(vector3d_vector3_msg);

  // gz.msgs.Wrench.
  auto wrench_wrench_pub =
    node.Advertise<gz::msgs::Wrench>("wrench_wrench");
  gz::msgs::Wrench wrench_wrench_msg;
  ros_gz_bridge::testing::createTestMsg(wrench_wrench_msg);

  // gz.msgs.Wrench.
  auto wrench_wrenchstamped_pub =
    node.Advertise<gz::msgs::Wrench>("wrench_wrenchstamped");
  gz::msgs::Wrench wrench_wrenchstamped_msg;
  ros_gz_bridge::testing::createTestMsg(wrench_wrenchstamped_msg);

  // gz.msgs.NavSat.
  auto navsat_gpsfix_pub =
    node.Advertise<gz::msgs::NavSat>("navsat_gpsfix");
  gz::msgs::NavSat navsat_gpsfix_msg;
  ros_gz_bridge::testing::createTestMsg(navsat_gpsfix_msg);

  // gz.msgs.Odometry.
  auto odometry_odometry_pub =
    node.Advertise<gz::msgs::Odometry>("odometry_odometry");
  gz::msgs::Odometry odometry_odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometry_odometry_msg);

  // gz.msgs.OdometryWithCovariance.
  auto odometrywithcovariance_odometry_pub =
    node.Advertise<gz::msgs::OdometryWithCovariance>("odometrywithcovariance_odometry");
  gz::msgs::OdometryWithCovariance odometrywithcovariance_odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometrywithcovariance_odometry_msg);

  // gz.msgs.Any.
  auto any_parametervalue_pub =
    node.Advertise<gz::msgs::Any>("any_parametervalue");
  gz::msgs::Any any_parametervalue_msg;
  ros_gz_bridge::testing::createTestMsg(any_parametervalue_msg);

  // gz.msgs.Altimeter.
  auto altimeter_altimeter_pub =
    node.Advertise<gz::msgs::Altimeter>("altimeter_altimeter");
  gz::msgs::Altimeter altimeter_altimeter_msg;
  ros_gz_bridge::testing::createTestMsg(altimeter_altimeter_msg);

  // gz.msgs.Contact.
  auto contact_contact_pub =
    node.Advertise<gz::msgs::Contact>("contact_contact");
  gz::msgs::Contact contact_contact_msg;
  ros_gz_bridge::testing::createTestMsg(contact_contact_msg);

  // gz.msgs.Contacts.
  auto contacts_contacts_pub =
    node.Advertise<gz::msgs::Contacts>("contacts_contacts");
  gz::msgs::Contacts contacts_contacts_msg;
  ros_gz_bridge::testing::createTestMsg(contacts_contacts_msg);

  // gz.msgs.Dataframe.
  auto dataframe_dataframe_pub =
    node.Advertise<gz::msgs::Dataframe>("dataframe_dataframe");
  gz::msgs::Dataframe dataframe_dataframe_msg;
  ros_gz_bridge::testing::createTestMsg(dataframe_dataframe_msg);

  // gz.msgs.Entity.
  auto entity_entity_pub =
    node.Advertise<gz::msgs::Entity>("entity_entity");
  gz::msgs::Entity entity_entity_msg;
  ros_gz_bridge::testing::createTestMsg(entity_entity_msg);

  // gz.msgs.EntityWrench.
  auto entitywrench_entitywrench_pub =
    node.Advertise<gz::msgs::EntityWrench>("entitywrench_entitywrench");
  gz::msgs::EntityWrench entitywrench_entitywrench_msg;
  ros_gz_bridge::testing::createTestMsg(entitywrench_entitywrench_msg);

  // gz.msgs.Float_V.
  auto float_v_float32array_pub =
    node.Advertise<gz::msgs::Float_V>("float_v_float32array");
  gz::msgs::Float_V float_v_float32array_msg;
  ros_gz_bridge::testing::createTestMsg(float_v_float32array_msg);

  // gz.msgs.GUICamera.
  auto guicamera_guicamera_pub =
    node.Advertise<gz::msgs::GUICamera>("guicamera_guicamera");
  gz::msgs::GUICamera guicamera_guicamera_msg;
  ros_gz_bridge::testing::createTestMsg(guicamera_guicamera_msg);

  // gz.msgs.JointWrench.
  auto jointwrench_jointwrench_pub =
    node.Advertise<gz::msgs::JointWrench>("jointwrench_jointwrench");
  gz::msgs::JointWrench jointwrench_jointwrench_msg;
  ros_gz_bridge::testing::createTestMsg(jointwrench_jointwrench_msg);

  // gz.msgs.Light.
  auto light_light_pub =
    node.Advertise<gz::msgs::Light>("light_light");
  gz::msgs::Light light_light_msg;
  ros_gz_bridge::testing::createTestMsg(light_light_msg);

  // gz.msgs.Param.
  auto param_paramvec_pub =
    node.Advertise<gz::msgs::Param>("param_paramvec");
  gz::msgs::Param param_paramvec_msg;
  ros_gz_bridge::testing::createTestMsg(param_paramvec_msg);

  // gz.msgs.Param_V.
  auto param_v_paramvec_pub =
    node.Advertise<gz::msgs::Param_V>("param_v_paramvec");
  gz::msgs::Param_V param_v_paramvec_msg;
  ros_gz_bridge::testing::createTestMsg(param_v_paramvec_msg);

  // gz.msgs.SensorNoise.
  auto sensornoise_sensornoise_pub =
    node.Advertise<gz::msgs::SensorNoise>("sensornoise_sensornoise");
  gz::msgs::SensorNoise sensornoise_sensornoise_msg;
  ros_gz_bridge::testing::createTestMsg(sensornoise_sensornoise_msg);

  // gz.msgs.StringMsg_V.
  auto stringmsg_v_stringvec_pub =
    node.Advertise<gz::msgs::StringMsg_V>("stringmsg_v_stringvec");
  gz::msgs::StringMsg_V stringmsg_v_stringvec_msg;
  ros_gz_bridge::testing::createTestMsg(stringmsg_v_stringvec_msg);

  // gz.msgs.TrackVisual.
  auto trackvisual_trackvisual_pub =
    node.Advertise<gz::msgs::TrackVisual>("trackvisual_trackvisual");
  gz::msgs::TrackVisual trackvisual_trackvisual_msg;
  ros_gz_bridge::testing::createTestMsg(trackvisual_trackvisual_msg);

  // gz.msgs.VideoRecord.
  auto videorecord_videorecord_pub =
    node.Advertise<gz::msgs::VideoRecord>("videorecord_videorecord");
  gz::msgs::VideoRecord videorecord_videorecord_msg;
  ros_gz_bridge::testing::createTestMsg(videorecord_videorecord_msg);

  // gz.msgs.Clock.
  auto clock_clock_pub =
    node.Advertise<gz::msgs::Clock>("clock_clock");
  gz::msgs::Clock clock_clock_msg;
  ros_gz_bridge::testing::createTestMsg(clock_clock_msg);

  // gz.msgs.BatteryState.
  auto batterystate_batterystate_pub =
    node.Advertise<gz::msgs::BatteryState>("batterystate_batterystate");
  gz::msgs::BatteryState batterystate_batterystate_msg;
  ros_gz_bridge::testing::createTestMsg(batterystate_batterystate_msg);

  // gz.msgs.CameraInfo.
  auto camerainfo_camerainfo_pub =
    node.Advertise<gz::msgs::CameraInfo>("camerainfo_camerainfo");
  gz::msgs::CameraInfo camerainfo_camerainfo_msg;
  ros_gz_bridge::testing::createTestMsg(camerainfo_camerainfo_msg);

  // gz.msgs.FluidPressure.
  auto fluidpressure_fluidpressure_pub =
    node.Advertise<gz::msgs::FluidPressure>("fluidpressure_fluidpressure");
  gz::msgs::FluidPressure fluidpressure_fluidpressure_msg;
  ros_gz_bridge::testing::createTestMsg(fluidpressure_fluidpressure_msg);

  // gz.msgs.Image.
  auto image_image_pub =
    node.Advertise<gz::msgs::Image>("image_image");
  gz::msgs::Image image_image_msg;
  ros_gz_bridge::testing::createTestMsg(image_image_msg);

  // gz.msgs.IMU.
  auto imu_imu_pub =
    node.Advertise<gz::msgs::IMU>("imu_imu");
  gz::msgs::IMU imu_imu_msg;
  ros_gz_bridge::testing::createTestMsg(imu_imu_msg);

  // gz.msgs.Model.
  auto model_jointstate_pub =
    node.Advertise<gz::msgs::Model>("model_jointstate");
  gz::msgs::Model model_jointstate_msg;
  ros_gz_bridge::testing::createTestMsg(model_jointstate_msg);

  // gz.msgs.Joy.
  auto joy_joy_pub =
    node.Advertise<gz::msgs::Joy>("joy_joy");
  gz::msgs::Joy joy_joy_msg;
  ros_gz_bridge::testing::createTestMsg(joy_joy_msg);

  // gz.msgs.LaserScan.
  auto laserscan_laserscan_pub =
    node.Advertise<gz::msgs::LaserScan>("laserscan_laserscan");
  gz::msgs::LaserScan laserscan_laserscan_msg;
  ros_gz_bridge::testing::createTestMsg(laserscan_laserscan_msg);

  // gz.msgs.Magnetometer.
  auto magnetometer_magneticfield_pub =
    node.Advertise<gz::msgs::Magnetometer>("magnetometer_magneticfield");
  gz::msgs::Magnetometer magnetometer_magneticfield_msg;
  ros_gz_bridge::testing::createTestMsg(magnetometer_magneticfield_msg);

  // gz.msgs.NavSat.
  auto navsat_navsatfix_pub =
    node.Advertise<gz::msgs::NavSat>("navsat_navsatfix");
  gz::msgs::NavSat navsat_navsatfix_msg;
  ros_gz_bridge::testing::createTestMsg(navsat_navsatfix_msg);

  // gz.msgs.PointCloudPacked.
  auto pointcloudpacked_pointcloud2_pub =
    node.Advertise<gz::msgs::PointCloudPacked>("pointcloudpacked_pointcloud2");
  gz::msgs::PointCloudPacked pointcloudpacked_pointcloud2_msg;
  ros_gz_bridge::testing::createTestMsg(pointcloudpacked_pointcloud2_msg);

  // gz.msgs.Boolean.
  auto boolean_bool_pub =
    node.Advertise<gz::msgs::Boolean>("boolean_bool");
  gz::msgs::Boolean boolean_bool_msg;
  ros_gz_bridge::testing::createTestMsg(boolean_bool_msg);

  // gz.msgs.Color.
  auto color_colorrgba_pub =
    node.Advertise<gz::msgs::Color>("color_colorrgba");
  gz::msgs::Color color_colorrgba_msg;
  ros_gz_bridge::testing::createTestMsg(color_colorrgba_msg);

  // gz.msgs.Empty.
  auto empty_empty_pub =
    node.Advertise<gz::msgs::Empty>("empty_empty");
  gz::msgs::Empty empty_empty_msg;
  ros_gz_bridge::testing::createTestMsg(empty_empty_msg);

  // gz.msgs.Float.
  auto float_float32_pub =
    node.Advertise<gz::msgs::Float>("float_float32");
  gz::msgs::Float float_float32_msg;
  ros_gz_bridge::testing::createTestMsg(float_float32_msg);

  // gz.msgs.Double.
  auto double_float64_pub =
    node.Advertise<gz::msgs::Double>("double_float64");
  gz::msgs::Double double_float64_msg;
  ros_gz_bridge::testing::createTestMsg(double_float64_msg);

  // gz.msgs.Header.
  auto header_header_pub =
    node.Advertise<gz::msgs::Header>("header_header");
  gz::msgs::Header header_header_msg;
  ros_gz_bridge::testing::createTestMsg(header_header_msg);

  // gz.msgs.Int32.
  auto int32_int32_pub =
    node.Advertise<gz::msgs::Int32>("int32_int32");
  gz::msgs::Int32 int32_int32_msg;
  ros_gz_bridge::testing::createTestMsg(int32_int32_msg);

  // gz.msgs.StringMsg.
  auto stringmsg_string_pub =
    node.Advertise<gz::msgs::StringMsg>("stringmsg_string");
  gz::msgs::StringMsg stringmsg_string_msg;
  ros_gz_bridge::testing::createTestMsg(stringmsg_string_msg);

  // gz.msgs.UInt32.
  auto uint32_uint32_pub =
    node.Advertise<gz::msgs::UInt32>("uint32_uint32");
  gz::msgs::UInt32 uint32_uint32_msg;
  ros_gz_bridge::testing::createTestMsg(uint32_uint32_msg);

  // gz.msgs.Pose_V.
  auto pose_v_tfmessage_pub =
    node.Advertise<gz::msgs::Pose_V>("pose_v_tfmessage");
  gz::msgs::Pose_V pose_v_tfmessage_msg;
  ros_gz_bridge::testing::createTestMsg(pose_v_tfmessage_msg);

  // gz.msgs.JointTrajectory.
  auto jointtrajectory_jointtrajectory_pub =
    node.Advertise<gz::msgs::JointTrajectory>("jointtrajectory_jointtrajectory");
  gz::msgs::JointTrajectory jointtrajectory_jointtrajectory_msg;
  ros_gz_bridge::testing::createTestMsg(jointtrajectory_jointtrajectory_msg);

  // gz.msgs.AnnotatedAxisAligned2DBox.
  auto annotatedaxisaligned2dbox_detection2d_pub =
    node.Advertise<gz::msgs::AnnotatedAxisAligned2DBox>("annotatedaxisaligned2dbox_detection2d");
  gz::msgs::AnnotatedAxisAligned2DBox annotatedaxisaligned2dbox_detection2d_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedaxisaligned2dbox_detection2d_msg);

  // gz.msgs.AnnotatedAxisAligned2DBox_V.
  auto annotatedaxisaligned2dbox_v_detection2darray_pub =
    node.Advertise<gz::msgs::AnnotatedAxisAligned2DBox_V>("annotatedaxisaligned2dbox_v_detection2darray");
  gz::msgs::AnnotatedAxisAligned2DBox_V annotatedaxisaligned2dbox_v_detection2darray_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedaxisaligned2dbox_v_detection2darray_msg);

  // gz.msgs.AnnotatedOriented3DBox.
  auto annotatedoriented3dbox_detection3d_pub =
    node.Advertise<gz::msgs::AnnotatedOriented3DBox>("annotatedoriented3dbox_detection3d");
  gz::msgs::AnnotatedOriented3DBox annotatedoriented3dbox_detection3d_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedoriented3dbox_detection3d_msg);

  // gz.msgs.AnnotatedOriented3DBox_V.
  auto annotatedoriented3dbox_v_detection3darray_pub =
    node.Advertise<gz::msgs::AnnotatedOriented3DBox_V>("annotatedoriented3dbox_v_detection3darray");
  gz::msgs::AnnotatedOriented3DBox_V annotatedoriented3dbox_v_detection3darray_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedoriented3dbox_v_detection3darray_msg);


  // Publish messages at 100 Hz.
  while (!g_terminatePub) {
    actuators_actuators_pub.Publish(actuators_actuators_msg);
    time_time_pub.Publish(time_time_msg);
    vector3d_point_pub.Publish(vector3d_point_msg);
    pose_pose_pub.Publish(pose_pose_msg);
    pose_v_posearray_pub.Publish(pose_v_posearray_msg);
    pose_posestamped_pub.Publish(pose_posestamped_msg);
    posewithcovariance_posewithcovariance_pub.Publish(posewithcovariance_posewithcovariance_msg);
    quaternion_quaternion_pub.Publish(quaternion_quaternion_msg);
    pose_transform_pub.Publish(pose_transform_msg);
    pose_transformstamped_pub.Publish(pose_transformstamped_msg);
    twist_twist_pub.Publish(twist_twist_msg);
    twist_twiststamped_pub.Publish(twist_twiststamped_msg);
    twistwithcovariance_twistwithcovariance_pub.Publish(twistwithcovariance_twistwithcovariance_msg);
    twistwithcovariance_twistwithcovariancestamped_pub.Publish(twistwithcovariance_twistwithcovariancestamped_msg);
    vector3d_vector3_pub.Publish(vector3d_vector3_msg);
    wrench_wrench_pub.Publish(wrench_wrench_msg);
    wrench_wrenchstamped_pub.Publish(wrench_wrenchstamped_msg);
    navsat_gpsfix_pub.Publish(navsat_gpsfix_msg);
    odometry_odometry_pub.Publish(odometry_odometry_msg);
    odometrywithcovariance_odometry_pub.Publish(odometrywithcovariance_odometry_msg);
    any_parametervalue_pub.Publish(any_parametervalue_msg);
    altimeter_altimeter_pub.Publish(altimeter_altimeter_msg);
    contact_contact_pub.Publish(contact_contact_msg);
    contacts_contacts_pub.Publish(contacts_contacts_msg);
    dataframe_dataframe_pub.Publish(dataframe_dataframe_msg);
    entity_entity_pub.Publish(entity_entity_msg);
    entitywrench_entitywrench_pub.Publish(entitywrench_entitywrench_msg);
    float_v_float32array_pub.Publish(float_v_float32array_msg);
    guicamera_guicamera_pub.Publish(guicamera_guicamera_msg);
    jointwrench_jointwrench_pub.Publish(jointwrench_jointwrench_msg);
    light_light_pub.Publish(light_light_msg);
    param_paramvec_pub.Publish(param_paramvec_msg);
    param_v_paramvec_pub.Publish(param_v_paramvec_msg);
    sensornoise_sensornoise_pub.Publish(sensornoise_sensornoise_msg);
    stringmsg_v_stringvec_pub.Publish(stringmsg_v_stringvec_msg);
    trackvisual_trackvisual_pub.Publish(trackvisual_trackvisual_msg);
    videorecord_videorecord_pub.Publish(videorecord_videorecord_msg);
    clock_clock_pub.Publish(clock_clock_msg);
    batterystate_batterystate_pub.Publish(batterystate_batterystate_msg);
    camerainfo_camerainfo_pub.Publish(camerainfo_camerainfo_msg);
    fluidpressure_fluidpressure_pub.Publish(fluidpressure_fluidpressure_msg);
    image_image_pub.Publish(image_image_msg);
    imu_imu_pub.Publish(imu_imu_msg);
    model_jointstate_pub.Publish(model_jointstate_msg);
    joy_joy_pub.Publish(joy_joy_msg);
    laserscan_laserscan_pub.Publish(laserscan_laserscan_msg);
    magnetometer_magneticfield_pub.Publish(magnetometer_magneticfield_msg);
    navsat_navsatfix_pub.Publish(navsat_navsatfix_msg);
    pointcloudpacked_pointcloud2_pub.Publish(pointcloudpacked_pointcloud2_msg);
    boolean_bool_pub.Publish(boolean_bool_msg);
    color_colorrgba_pub.Publish(color_colorrgba_msg);
    empty_empty_pub.Publish(empty_empty_msg);
    float_float32_pub.Publish(float_float32_msg);
    double_float64_pub.Publish(double_float64_msg);
    header_header_pub.Publish(header_header_msg);
    int32_int32_pub.Publish(int32_int32_msg);
    stringmsg_string_pub.Publish(stringmsg_string_msg);
    uint32_uint32_pub.Publish(uint32_uint32_msg);
    pose_v_tfmessage_pub.Publish(pose_v_tfmessage_msg);
    jointtrajectory_jointtrajectory_pub.Publish(jointtrajectory_jointtrajectory_msg);
    annotatedaxisaligned2dbox_detection2d_pub.Publish(annotatedaxisaligned2dbox_detection2d_msg);
    annotatedaxisaligned2dbox_v_detection2darray_pub.Publish(annotatedaxisaligned2dbox_v_detection2darray_msg);
    annotatedoriented3dbox_detection3d_pub.Publish(annotatedoriented3dbox_detection3d_msg);
    annotatedoriented3dbox_v_detection3darray_pub.Publish(annotatedoriented3dbox_v_detection3darray_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
