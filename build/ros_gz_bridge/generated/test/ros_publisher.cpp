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

#include <rclcpp/rclcpp.hpp>
#include "utils/test_utils.hpp"
#include "utils/ros_test_msg.hpp"

//////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_string_publisher");
  rclcpp::Rate loop_rate(100);

  // actuator_msgs/msg/Actuators.
  auto actuators_actuators_pub =
    node->create_publisher<actuator_msgs::msg::Actuators>("actuators_actuators", 1);
  actuator_msgs::msg::Actuators actuators_actuators_msg;
  ros_gz_bridge::testing::createTestMsg(actuators_actuators_msg);

  // builtin_interfaces/msg/Time.
  auto time_time_pub =
    node->create_publisher<builtin_interfaces::msg::Time>("time_time", 1);
  builtin_interfaces::msg::Time time_time_msg;
  ros_gz_bridge::testing::createTestMsg(time_time_msg);

  // geometry_msgs/msg/Point.
  auto vector3d_point_pub =
    node->create_publisher<geometry_msgs::msg::Point>("vector3d_point", 1);
  geometry_msgs::msg::Point vector3d_point_msg;
  ros_gz_bridge::testing::createTestMsg(vector3d_point_msg);

  // geometry_msgs/msg/Pose.
  auto pose_pose_pub =
    node->create_publisher<geometry_msgs::msg::Pose>("pose_pose", 1);
  geometry_msgs::msg::Pose pose_pose_msg;
  ros_gz_bridge::testing::createTestMsg(pose_pose_msg);

  // geometry_msgs/msg/PoseArray.
  auto pose_v_posearray_pub =
    node->create_publisher<geometry_msgs::msg::PoseArray>("pose_v_posearray", 1);
  geometry_msgs::msg::PoseArray pose_v_posearray_msg;
  ros_gz_bridge::testing::createTestMsg(pose_v_posearray_msg);

  // geometry_msgs/msg/PoseStamped.
  auto pose_posestamped_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("pose_posestamped", 1);
  geometry_msgs::msg::PoseStamped pose_posestamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_posestamped_msg);

  // geometry_msgs/msg/PoseWithCovariance.
  auto posewithcovariance_posewithcovariance_pub =
    node->create_publisher<geometry_msgs::msg::PoseWithCovariance>("posewithcovariance_posewithcovariance", 1);
  geometry_msgs::msg::PoseWithCovariance posewithcovariance_posewithcovariance_msg;
  ros_gz_bridge::testing::createTestMsg(posewithcovariance_posewithcovariance_msg);

  // geometry_msgs/msg/Quaternion.
  auto quaternion_quaternion_pub =
    node->create_publisher<geometry_msgs::msg::Quaternion>("quaternion_quaternion", 1);
  geometry_msgs::msg::Quaternion quaternion_quaternion_msg;
  ros_gz_bridge::testing::createTestMsg(quaternion_quaternion_msg);

  // geometry_msgs/msg/Transform.
  auto pose_transform_pub =
    node->create_publisher<geometry_msgs::msg::Transform>("pose_transform", 1);
  geometry_msgs::msg::Transform pose_transform_msg;
  ros_gz_bridge::testing::createTestMsg(pose_transform_msg);

  // geometry_msgs/msg/TransformStamped.
  auto pose_transformstamped_pub =
    node->create_publisher<geometry_msgs::msg::TransformStamped>("pose_transformstamped", 1);
  geometry_msgs::msg::TransformStamped pose_transformstamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_transformstamped_msg);

  // geometry_msgs/msg/Twist.
  auto twist_twist_pub =
    node->create_publisher<geometry_msgs::msg::Twist>("twist_twist", 1);
  geometry_msgs::msg::Twist twist_twist_msg;
  ros_gz_bridge::testing::createTestMsg(twist_twist_msg);

  // geometry_msgs/msg/TwistStamped.
  auto twist_twiststamped_pub =
    node->create_publisher<geometry_msgs::msg::TwistStamped>("twist_twiststamped", 1);
  geometry_msgs::msg::TwistStamped twist_twiststamped_msg;
  ros_gz_bridge::testing::createTestMsg(twist_twiststamped_msg);

  // geometry_msgs/msg/TwistWithCovariance.
  auto twistwithcovariance_twistwithcovariance_pub =
    node->create_publisher<geometry_msgs::msg::TwistWithCovariance>("twistwithcovariance_twistwithcovariance", 1);
  geometry_msgs::msg::TwistWithCovariance twistwithcovariance_twistwithcovariance_msg;
  ros_gz_bridge::testing::createTestMsg(twistwithcovariance_twistwithcovariance_msg);

  // geometry_msgs/msg/TwistWithCovarianceStamped.
  auto twistwithcovariance_twistwithcovariancestamped_pub =
    node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twistwithcovariance_twistwithcovariancestamped", 1);
  geometry_msgs::msg::TwistWithCovarianceStamped twistwithcovariance_twistwithcovariancestamped_msg;
  ros_gz_bridge::testing::createTestMsg(twistwithcovariance_twistwithcovariancestamped_msg);

  // geometry_msgs/msg/Vector3.
  auto vector3d_vector3_pub =
    node->create_publisher<geometry_msgs::msg::Vector3>("vector3d_vector3", 1);
  geometry_msgs::msg::Vector3 vector3d_vector3_msg;
  ros_gz_bridge::testing::createTestMsg(vector3d_vector3_msg);

  // geometry_msgs/msg/Wrench.
  auto wrench_wrench_pub =
    node->create_publisher<geometry_msgs::msg::Wrench>("wrench_wrench", 1);
  geometry_msgs::msg::Wrench wrench_wrench_msg;
  ros_gz_bridge::testing::createTestMsg(wrench_wrench_msg);

  // geometry_msgs/msg/WrenchStamped.
  auto wrench_wrenchstamped_pub =
    node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_wrenchstamped", 1);
  geometry_msgs::msg::WrenchStamped wrench_wrenchstamped_msg;
  ros_gz_bridge::testing::createTestMsg(wrench_wrenchstamped_msg);

  // gps_msgs/msg/GPSFix.
  auto navsat_gpsfix_pub =
    node->create_publisher<gps_msgs::msg::GPSFix>("navsat_gpsfix", 1);
  gps_msgs::msg::GPSFix navsat_gpsfix_msg;
  ros_gz_bridge::testing::createTestMsg(navsat_gpsfix_msg);

  // nav_msgs/msg/Odometry.
  auto odometry_odometry_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("odometry_odometry", 1);
  nav_msgs::msg::Odometry odometry_odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometry_odometry_msg);

  // nav_msgs/msg/Odometry.
  auto odometrywithcovariance_odometry_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("odometrywithcovariance_odometry", 1);
  nav_msgs::msg::Odometry odometrywithcovariance_odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometrywithcovariance_odometry_msg);

  // rcl_interfaces/msg/ParameterValue.
  auto any_parametervalue_pub =
    node->create_publisher<rcl_interfaces::msg::ParameterValue>("any_parametervalue", 1);
  rcl_interfaces::msg::ParameterValue any_parametervalue_msg;
  ros_gz_bridge::testing::createTestMsg(any_parametervalue_msg);

  // ros_gz_interfaces/msg/Altimeter.
  auto altimeter_altimeter_pub =
    node->create_publisher<ros_gz_interfaces::msg::Altimeter>("altimeter_altimeter", 1);
  ros_gz_interfaces::msg::Altimeter altimeter_altimeter_msg;
  ros_gz_bridge::testing::createTestMsg(altimeter_altimeter_msg);

  // ros_gz_interfaces/msg/Contact.
  auto contact_contact_pub =
    node->create_publisher<ros_gz_interfaces::msg::Contact>("contact_contact", 1);
  ros_gz_interfaces::msg::Contact contact_contact_msg;
  ros_gz_bridge::testing::createTestMsg(contact_contact_msg);

  // ros_gz_interfaces/msg/Contacts.
  auto contacts_contacts_pub =
    node->create_publisher<ros_gz_interfaces::msg::Contacts>("contacts_contacts", 1);
  ros_gz_interfaces::msg::Contacts contacts_contacts_msg;
  ros_gz_bridge::testing::createTestMsg(contacts_contacts_msg);

  // ros_gz_interfaces/msg/Dataframe.
  auto dataframe_dataframe_pub =
    node->create_publisher<ros_gz_interfaces::msg::Dataframe>("dataframe_dataframe", 1);
  ros_gz_interfaces::msg::Dataframe dataframe_dataframe_msg;
  ros_gz_bridge::testing::createTestMsg(dataframe_dataframe_msg);

  // ros_gz_interfaces/msg/Entity.
  auto entity_entity_pub =
    node->create_publisher<ros_gz_interfaces::msg::Entity>("entity_entity", 1);
  ros_gz_interfaces::msg::Entity entity_entity_msg;
  ros_gz_bridge::testing::createTestMsg(entity_entity_msg);

  // ros_gz_interfaces/msg/EntityWrench.
  auto entitywrench_entitywrench_pub =
    node->create_publisher<ros_gz_interfaces::msg::EntityWrench>("entitywrench_entitywrench", 1);
  ros_gz_interfaces::msg::EntityWrench entitywrench_entitywrench_msg;
  ros_gz_bridge::testing::createTestMsg(entitywrench_entitywrench_msg);

  // ros_gz_interfaces/msg/Float32Array.
  auto float_v_float32array_pub =
    node->create_publisher<ros_gz_interfaces::msg::Float32Array>("float_v_float32array", 1);
  ros_gz_interfaces::msg::Float32Array float_v_float32array_msg;
  ros_gz_bridge::testing::createTestMsg(float_v_float32array_msg);

  // ros_gz_interfaces/msg/GuiCamera.
  auto guicamera_guicamera_pub =
    node->create_publisher<ros_gz_interfaces::msg::GuiCamera>("guicamera_guicamera", 1);
  ros_gz_interfaces::msg::GuiCamera guicamera_guicamera_msg;
  ros_gz_bridge::testing::createTestMsg(guicamera_guicamera_msg);

  // ros_gz_interfaces/msg/JointWrench.
  auto jointwrench_jointwrench_pub =
    node->create_publisher<ros_gz_interfaces::msg::JointWrench>("jointwrench_jointwrench", 1);
  ros_gz_interfaces::msg::JointWrench jointwrench_jointwrench_msg;
  ros_gz_bridge::testing::createTestMsg(jointwrench_jointwrench_msg);

  // ros_gz_interfaces/msg/Light.
  auto light_light_pub =
    node->create_publisher<ros_gz_interfaces::msg::Light>("light_light", 1);
  ros_gz_interfaces::msg::Light light_light_msg;
  ros_gz_bridge::testing::createTestMsg(light_light_msg);

  // ros_gz_interfaces/msg/ParamVec.
  auto param_paramvec_pub =
    node->create_publisher<ros_gz_interfaces::msg::ParamVec>("param_paramvec", 1);
  ros_gz_interfaces::msg::ParamVec param_paramvec_msg;
  ros_gz_bridge::testing::createTestMsg(param_paramvec_msg);

  // ros_gz_interfaces/msg/ParamVec.
  auto param_v_paramvec_pub =
    node->create_publisher<ros_gz_interfaces::msg::ParamVec>("param_v_paramvec", 1);
  ros_gz_interfaces::msg::ParamVec param_v_paramvec_msg;
  ros_gz_bridge::testing::createTestMsg(param_v_paramvec_msg);

  // ros_gz_interfaces/msg/SensorNoise.
  auto sensornoise_sensornoise_pub =
    node->create_publisher<ros_gz_interfaces::msg::SensorNoise>("sensornoise_sensornoise", 1);
  ros_gz_interfaces::msg::SensorNoise sensornoise_sensornoise_msg;
  ros_gz_bridge::testing::createTestMsg(sensornoise_sensornoise_msg);

  // ros_gz_interfaces/msg/StringVec.
  auto stringmsg_v_stringvec_pub =
    node->create_publisher<ros_gz_interfaces::msg::StringVec>("stringmsg_v_stringvec", 1);
  ros_gz_interfaces::msg::StringVec stringmsg_v_stringvec_msg;
  ros_gz_bridge::testing::createTestMsg(stringmsg_v_stringvec_msg);

  // ros_gz_interfaces/msg/TrackVisual.
  auto trackvisual_trackvisual_pub =
    node->create_publisher<ros_gz_interfaces::msg::TrackVisual>("trackvisual_trackvisual", 1);
  ros_gz_interfaces::msg::TrackVisual trackvisual_trackvisual_msg;
  ros_gz_bridge::testing::createTestMsg(trackvisual_trackvisual_msg);

  // ros_gz_interfaces/msg/VideoRecord.
  auto videorecord_videorecord_pub =
    node->create_publisher<ros_gz_interfaces::msg::VideoRecord>("videorecord_videorecord", 1);
  ros_gz_interfaces::msg::VideoRecord videorecord_videorecord_msg;
  ros_gz_bridge::testing::createTestMsg(videorecord_videorecord_msg);

  // rosgraph_msgs/msg/Clock.
  auto clock_clock_pub =
    node->create_publisher<rosgraph_msgs::msg::Clock>("clock_clock", 1);
  rosgraph_msgs::msg::Clock clock_clock_msg;
  ros_gz_bridge::testing::createTestMsg(clock_clock_msg);

  // sensor_msgs/msg/BatteryState.
  auto batterystate_batterystate_pub =
    node->create_publisher<sensor_msgs::msg::BatteryState>("batterystate_batterystate", 1);
  sensor_msgs::msg::BatteryState batterystate_batterystate_msg;
  ros_gz_bridge::testing::createTestMsg(batterystate_batterystate_msg);

  // sensor_msgs/msg/CameraInfo.
  auto camerainfo_camerainfo_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("camerainfo_camerainfo", 1);
  sensor_msgs::msg::CameraInfo camerainfo_camerainfo_msg;
  ros_gz_bridge::testing::createTestMsg(camerainfo_camerainfo_msg);

  // sensor_msgs/msg/FluidPressure.
  auto fluidpressure_fluidpressure_pub =
    node->create_publisher<sensor_msgs::msg::FluidPressure>("fluidpressure_fluidpressure", 1);
  sensor_msgs::msg::FluidPressure fluidpressure_fluidpressure_msg;
  ros_gz_bridge::testing::createTestMsg(fluidpressure_fluidpressure_msg);

  // sensor_msgs/msg/Image.
  auto image_image_pub =
    node->create_publisher<sensor_msgs::msg::Image>("image_image", 1);
  sensor_msgs::msg::Image image_image_msg;
  ros_gz_bridge::testing::createTestMsg(image_image_msg);

  // sensor_msgs/msg/Imu.
  auto imu_imu_pub =
    node->create_publisher<sensor_msgs::msg::Imu>("imu_imu", 1);
  sensor_msgs::msg::Imu imu_imu_msg;
  ros_gz_bridge::testing::createTestMsg(imu_imu_msg);

  // sensor_msgs/msg/JointState.
  auto model_jointstate_pub =
    node->create_publisher<sensor_msgs::msg::JointState>("model_jointstate", 1);
  sensor_msgs::msg::JointState model_jointstate_msg;
  ros_gz_bridge::testing::createTestMsg(model_jointstate_msg);

  // sensor_msgs/msg/Joy.
  auto joy_joy_pub =
    node->create_publisher<sensor_msgs::msg::Joy>("joy_joy", 1);
  sensor_msgs::msg::Joy joy_joy_msg;
  ros_gz_bridge::testing::createTestMsg(joy_joy_msg);

  // sensor_msgs/msg/LaserScan.
  auto laserscan_laserscan_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>("laserscan_laserscan", 1);
  sensor_msgs::msg::LaserScan laserscan_laserscan_msg;
  ros_gz_bridge::testing::createTestMsg(laserscan_laserscan_msg);

  // sensor_msgs/msg/MagneticField.
  auto magnetometer_magneticfield_pub =
    node->create_publisher<sensor_msgs::msg::MagneticField>("magnetometer_magneticfield", 1);
  sensor_msgs::msg::MagneticField magnetometer_magneticfield_msg;
  ros_gz_bridge::testing::createTestMsg(magnetometer_magneticfield_msg);

  // sensor_msgs/msg/NavSatFix.
  auto navsat_navsatfix_pub =
    node->create_publisher<sensor_msgs::msg::NavSatFix>("navsat_navsatfix", 1);
  sensor_msgs::msg::NavSatFix navsat_navsatfix_msg;
  ros_gz_bridge::testing::createTestMsg(navsat_navsatfix_msg);

  // sensor_msgs/msg/PointCloud2.
  auto pointcloudpacked_pointcloud2_pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloudpacked_pointcloud2", 1);
  sensor_msgs::msg::PointCloud2 pointcloudpacked_pointcloud2_msg;
  ros_gz_bridge::testing::createTestMsg(pointcloudpacked_pointcloud2_msg);

  // std_msgs/msg/Bool.
  auto boolean_bool_pub =
    node->create_publisher<std_msgs::msg::Bool>("boolean_bool", 1);
  std_msgs::msg::Bool boolean_bool_msg;
  ros_gz_bridge::testing::createTestMsg(boolean_bool_msg);

  // std_msgs/msg/ColorRGBA.
  auto color_colorrgba_pub =
    node->create_publisher<std_msgs::msg::ColorRGBA>("color_colorrgba", 1);
  std_msgs::msg::ColorRGBA color_colorrgba_msg;
  ros_gz_bridge::testing::createTestMsg(color_colorrgba_msg);

  // std_msgs/msg/Empty.
  auto empty_empty_pub =
    node->create_publisher<std_msgs::msg::Empty>("empty_empty", 1);
  std_msgs::msg::Empty empty_empty_msg;
  ros_gz_bridge::testing::createTestMsg(empty_empty_msg);

  // std_msgs/msg/Float32.
  auto float_float32_pub =
    node->create_publisher<std_msgs::msg::Float32>("float_float32", 1);
  std_msgs::msg::Float32 float_float32_msg;
  ros_gz_bridge::testing::createTestMsg(float_float32_msg);

  // std_msgs/msg/Float64.
  auto double_float64_pub =
    node->create_publisher<std_msgs::msg::Float64>("double_float64", 1);
  std_msgs::msg::Float64 double_float64_msg;
  ros_gz_bridge::testing::createTestMsg(double_float64_msg);

  // std_msgs/msg/Header.
  auto header_header_pub =
    node->create_publisher<std_msgs::msg::Header>("header_header", 1);
  std_msgs::msg::Header header_header_msg;
  ros_gz_bridge::testing::createTestMsg(header_header_msg);

  // std_msgs/msg/Int32.
  auto int32_int32_pub =
    node->create_publisher<std_msgs::msg::Int32>("int32_int32", 1);
  std_msgs::msg::Int32 int32_int32_msg;
  ros_gz_bridge::testing::createTestMsg(int32_int32_msg);

  // std_msgs/msg/String.
  auto stringmsg_string_pub =
    node->create_publisher<std_msgs::msg::String>("stringmsg_string", 1);
  std_msgs::msg::String stringmsg_string_msg;
  ros_gz_bridge::testing::createTestMsg(stringmsg_string_msg);

  // std_msgs/msg/UInt32.
  auto uint32_uint32_pub =
    node->create_publisher<std_msgs::msg::UInt32>("uint32_uint32", 1);
  std_msgs::msg::UInt32 uint32_uint32_msg;
  ros_gz_bridge::testing::createTestMsg(uint32_uint32_msg);

  // tf2_msgs/msg/TFMessage.
  auto pose_v_tfmessage_pub =
    node->create_publisher<tf2_msgs::msg::TFMessage>("pose_v_tfmessage", 1);
  tf2_msgs::msg::TFMessage pose_v_tfmessage_msg;
  ros_gz_bridge::testing::createTestMsg(pose_v_tfmessage_msg);

  // trajectory_msgs/msg/JointTrajectory.
  auto jointtrajectory_jointtrajectory_pub =
    node->create_publisher<trajectory_msgs::msg::JointTrajectory>("jointtrajectory_jointtrajectory", 1);
  trajectory_msgs::msg::JointTrajectory jointtrajectory_jointtrajectory_msg;
  ros_gz_bridge::testing::createTestMsg(jointtrajectory_jointtrajectory_msg);

  // vision_msgs/msg/Detection2D.
  auto annotatedaxisaligned2dbox_detection2d_pub =
    node->create_publisher<vision_msgs::msg::Detection2D>("annotatedaxisaligned2dbox_detection2d", 1);
  vision_msgs::msg::Detection2D annotatedaxisaligned2dbox_detection2d_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedaxisaligned2dbox_detection2d_msg);

  // vision_msgs/msg/Detection2DArray.
  auto annotatedaxisaligned2dbox_v_detection2darray_pub =
    node->create_publisher<vision_msgs::msg::Detection2DArray>("annotatedaxisaligned2dbox_v_detection2darray", 1);
  vision_msgs::msg::Detection2DArray annotatedaxisaligned2dbox_v_detection2darray_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedaxisaligned2dbox_v_detection2darray_msg);

  // vision_msgs/msg/Detection3D.
  auto annotatedoriented3dbox_detection3d_pub =
    node->create_publisher<vision_msgs::msg::Detection3D>("annotatedoriented3dbox_detection3d", 1);
  vision_msgs::msg::Detection3D annotatedoriented3dbox_detection3d_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedoriented3dbox_detection3d_msg);

  // vision_msgs/msg/Detection3DArray.
  auto annotatedoriented3dbox_v_detection3darray_pub =
    node->create_publisher<vision_msgs::msg::Detection3DArray>("annotatedoriented3dbox_v_detection3darray", 1);
  vision_msgs::msg::Detection3DArray annotatedoriented3dbox_v_detection3darray_msg;
  ros_gz_bridge::testing::createTestMsg(annotatedoriented3dbox_v_detection3darray_msg);


  while (rclcpp::ok()) {
    // Publish all messages.
    actuators_actuators_pub->publish(actuators_actuators_msg);
    time_time_pub->publish(time_time_msg);
    vector3d_point_pub->publish(vector3d_point_msg);
    pose_pose_pub->publish(pose_pose_msg);
    pose_v_posearray_pub->publish(pose_v_posearray_msg);
    pose_posestamped_pub->publish(pose_posestamped_msg);
    posewithcovariance_posewithcovariance_pub->publish(posewithcovariance_posewithcovariance_msg);
    quaternion_quaternion_pub->publish(quaternion_quaternion_msg);
    pose_transform_pub->publish(pose_transform_msg);
    pose_transformstamped_pub->publish(pose_transformstamped_msg);
    twist_twist_pub->publish(twist_twist_msg);
    twist_twiststamped_pub->publish(twist_twiststamped_msg);
    twistwithcovariance_twistwithcovariance_pub->publish(twistwithcovariance_twistwithcovariance_msg);
    twistwithcovariance_twistwithcovariancestamped_pub->publish(twistwithcovariance_twistwithcovariancestamped_msg);
    vector3d_vector3_pub->publish(vector3d_vector3_msg);
    wrench_wrench_pub->publish(wrench_wrench_msg);
    wrench_wrenchstamped_pub->publish(wrench_wrenchstamped_msg);
    navsat_gpsfix_pub->publish(navsat_gpsfix_msg);
    odometry_odometry_pub->publish(odometry_odometry_msg);
    odometrywithcovariance_odometry_pub->publish(odometrywithcovariance_odometry_msg);
    any_parametervalue_pub->publish(any_parametervalue_msg);
    altimeter_altimeter_pub->publish(altimeter_altimeter_msg);
    contact_contact_pub->publish(contact_contact_msg);
    contacts_contacts_pub->publish(contacts_contacts_msg);
    dataframe_dataframe_pub->publish(dataframe_dataframe_msg);
    entity_entity_pub->publish(entity_entity_msg);
    entitywrench_entitywrench_pub->publish(entitywrench_entitywrench_msg);
    float_v_float32array_pub->publish(float_v_float32array_msg);
    guicamera_guicamera_pub->publish(guicamera_guicamera_msg);
    jointwrench_jointwrench_pub->publish(jointwrench_jointwrench_msg);
    light_light_pub->publish(light_light_msg);
    param_paramvec_pub->publish(param_paramvec_msg);
    param_v_paramvec_pub->publish(param_v_paramvec_msg);
    sensornoise_sensornoise_pub->publish(sensornoise_sensornoise_msg);
    stringmsg_v_stringvec_pub->publish(stringmsg_v_stringvec_msg);
    trackvisual_trackvisual_pub->publish(trackvisual_trackvisual_msg);
    videorecord_videorecord_pub->publish(videorecord_videorecord_msg);
    clock_clock_pub->publish(clock_clock_msg);
    batterystate_batterystate_pub->publish(batterystate_batterystate_msg);
    camerainfo_camerainfo_pub->publish(camerainfo_camerainfo_msg);
    fluidpressure_fluidpressure_pub->publish(fluidpressure_fluidpressure_msg);
    image_image_pub->publish(image_image_msg);
    imu_imu_pub->publish(imu_imu_msg);
    model_jointstate_pub->publish(model_jointstate_msg);
    joy_joy_pub->publish(joy_joy_msg);
    laserscan_laserscan_pub->publish(laserscan_laserscan_msg);
    magnetometer_magneticfield_pub->publish(magnetometer_magneticfield_msg);
    navsat_navsatfix_pub->publish(navsat_navsatfix_msg);
    pointcloudpacked_pointcloud2_pub->publish(pointcloudpacked_pointcloud2_msg);
    boolean_bool_pub->publish(boolean_bool_msg);
    color_colorrgba_pub->publish(color_colorrgba_msg);
    empty_empty_pub->publish(empty_empty_msg);
    float_float32_pub->publish(float_float32_msg);
    double_float64_pub->publish(double_float64_msg);
    header_header_pub->publish(header_header_msg);
    int32_int32_pub->publish(int32_int32_msg);
    stringmsg_string_pub->publish(stringmsg_string_msg);
    uint32_uint32_pub->publish(uint32_uint32_msg);
    pose_v_tfmessage_pub->publish(pose_v_tfmessage_msg);
    jointtrajectory_jointtrajectory_pub->publish(jointtrajectory_jointtrajectory_msg);
    annotatedaxisaligned2dbox_detection2d_pub->publish(annotatedaxisaligned2dbox_detection2d_msg);
    annotatedaxisaligned2dbox_v_detection2darray_pub->publish(annotatedaxisaligned2dbox_v_detection2darray_msg);
    annotatedoriented3dbox_detection3d_pub->publish(annotatedoriented3dbox_detection3d_msg);
    annotatedoriented3dbox_v_detection3darray_pub->publish(annotatedoriented3dbox_v_detection3darray_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
