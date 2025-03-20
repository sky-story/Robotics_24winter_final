// generated from ros_gz_bridge/resource/get_mappings.cpp.em

#include <map>
#include <string>

#include "get_mappings.hpp"

namespace ros_gz_bridge
{

bool
get_gz_to_ros_mapping(const std::string & gz_type_name, std::string & ros_type_name)
{

  if (gz_type_name == "gz.msgs.Actuators" || gz_type_name == "ignition.msgs.Actuators")
  {
    ros_type_name = "actuator_msgs/msg/Actuators";
    return true;
  }
  if (gz_type_name == "gz.msgs.Time" || gz_type_name == "ignition.msgs.Time")
  {
    ros_type_name = "builtin_interfaces/msg/Time";
    return true;
  }
  if (gz_type_name == "gz.msgs.Vector3d" || gz_type_name == "ignition.msgs.Vector3d")
  {
    ros_type_name = "geometry_msgs/msg/Point";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose")
  {
    ros_type_name = "geometry_msgs/msg/Pose";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose_V" || gz_type_name == "ignition.msgs.Pose_V")
  {
    ros_type_name = "geometry_msgs/msg/PoseArray";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose")
  {
    ros_type_name = "geometry_msgs/msg/PoseStamped";
    return true;
  }
  if (gz_type_name == "gz.msgs.PoseWithCovariance" || gz_type_name == "ignition.msgs.PoseWithCovariance")
  {
    ros_type_name = "geometry_msgs/msg/PoseWithCovariance";
    return true;
  }
  if (gz_type_name == "gz.msgs.Quaternion" || gz_type_name == "ignition.msgs.Quaternion")
  {
    ros_type_name = "geometry_msgs/msg/Quaternion";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose")
  {
    ros_type_name = "geometry_msgs/msg/Transform";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose")
  {
    ros_type_name = "geometry_msgs/msg/TransformStamped";
    return true;
  }
  if (gz_type_name == "gz.msgs.Twist" || gz_type_name == "ignition.msgs.Twist")
  {
    ros_type_name = "geometry_msgs/msg/Twist";
    return true;
  }
  if (gz_type_name == "gz.msgs.Twist" || gz_type_name == "ignition.msgs.Twist")
  {
    ros_type_name = "geometry_msgs/msg/TwistStamped";
    return true;
  }
  if (gz_type_name == "gz.msgs.TwistWithCovariance" || gz_type_name == "ignition.msgs.TwistWithCovariance")
  {
    ros_type_name = "geometry_msgs/msg/TwistWithCovariance";
    return true;
  }
  if (gz_type_name == "gz.msgs.TwistWithCovariance" || gz_type_name == "ignition.msgs.TwistWithCovariance")
  {
    ros_type_name = "geometry_msgs/msg/TwistWithCovarianceStamped";
    return true;
  }
  if (gz_type_name == "gz.msgs.Vector3d" || gz_type_name == "ignition.msgs.Vector3d")
  {
    ros_type_name = "geometry_msgs/msg/Vector3";
    return true;
  }
  if (gz_type_name == "gz.msgs.Wrench" || gz_type_name == "ignition.msgs.Wrench")
  {
    ros_type_name = "geometry_msgs/msg/Wrench";
    return true;
  }
  if (gz_type_name == "gz.msgs.Wrench" || gz_type_name == "ignition.msgs.Wrench")
  {
    ros_type_name = "geometry_msgs/msg/WrenchStamped";
    return true;
  }
  if (gz_type_name == "gz.msgs.NavSat" || gz_type_name == "ignition.msgs.NavSat")
  {
    ros_type_name = "gps_msgs/msg/GPSFix";
    return true;
  }
  if (gz_type_name == "gz.msgs.Odometry" || gz_type_name == "ignition.msgs.Odometry")
  {
    ros_type_name = "nav_msgs/msg/Odometry";
    return true;
  }
  if (gz_type_name == "gz.msgs.OdometryWithCovariance" || gz_type_name == "ignition.msgs.OdometryWithCovariance")
  {
    ros_type_name = "nav_msgs/msg/Odometry";
    return true;
  }
  if (gz_type_name == "gz.msgs.Any" || gz_type_name == "ignition.msgs.Any")
  {
    ros_type_name = "rcl_interfaces/msg/ParameterValue";
    return true;
  }
  if (gz_type_name == "gz.msgs.Altimeter" || gz_type_name == "ignition.msgs.Altimeter")
  {
    ros_type_name = "ros_gz_interfaces/msg/Altimeter";
    return true;
  }
  if (gz_type_name == "gz.msgs.Contact" || gz_type_name == "ignition.msgs.Contact")
  {
    ros_type_name = "ros_gz_interfaces/msg/Contact";
    return true;
  }
  if (gz_type_name == "gz.msgs.Contacts" || gz_type_name == "ignition.msgs.Contacts")
  {
    ros_type_name = "ros_gz_interfaces/msg/Contacts";
    return true;
  }
  if (gz_type_name == "gz.msgs.Dataframe" || gz_type_name == "ignition.msgs.Dataframe")
  {
    ros_type_name = "ros_gz_interfaces/msg/Dataframe";
    return true;
  }
  if (gz_type_name == "gz.msgs.Entity" || gz_type_name == "ignition.msgs.Entity")
  {
    ros_type_name = "ros_gz_interfaces/msg/Entity";
    return true;
  }
  if (gz_type_name == "gz.msgs.EntityWrench" || gz_type_name == "ignition.msgs.EntityWrench")
  {
    ros_type_name = "ros_gz_interfaces/msg/EntityWrench";
    return true;
  }
  if (gz_type_name == "gz.msgs.Float_V" || gz_type_name == "ignition.msgs.Float_V")
  {
    ros_type_name = "ros_gz_interfaces/msg/Float32Array";
    return true;
  }
  if (gz_type_name == "gz.msgs.GUICamera" || gz_type_name == "ignition.msgs.GUICamera")
  {
    ros_type_name = "ros_gz_interfaces/msg/GuiCamera";
    return true;
  }
  if (gz_type_name == "gz.msgs.JointWrench" || gz_type_name == "ignition.msgs.JointWrench")
  {
    ros_type_name = "ros_gz_interfaces/msg/JointWrench";
    return true;
  }
  if (gz_type_name == "gz.msgs.Light" || gz_type_name == "ignition.msgs.Light")
  {
    ros_type_name = "ros_gz_interfaces/msg/Light";
    return true;
  }
  if (gz_type_name == "gz.msgs.Param" || gz_type_name == "ignition.msgs.Param")
  {
    ros_type_name = "ros_gz_interfaces/msg/ParamVec";
    return true;
  }
  if (gz_type_name == "gz.msgs.Param_V" || gz_type_name == "ignition.msgs.Param_V")
  {
    ros_type_name = "ros_gz_interfaces/msg/ParamVec";
    return true;
  }
  if (gz_type_name == "gz.msgs.SensorNoise" || gz_type_name == "ignition.msgs.SensorNoise")
  {
    ros_type_name = "ros_gz_interfaces/msg/SensorNoise";
    return true;
  }
  if (gz_type_name == "gz.msgs.StringMsg_V" || gz_type_name == "ignition.msgs.StringMsg_V")
  {
    ros_type_name = "ros_gz_interfaces/msg/StringVec";
    return true;
  }
  if (gz_type_name == "gz.msgs.TrackVisual" || gz_type_name == "ignition.msgs.TrackVisual")
  {
    ros_type_name = "ros_gz_interfaces/msg/TrackVisual";
    return true;
  }
  if (gz_type_name == "gz.msgs.VideoRecord" || gz_type_name == "ignition.msgs.VideoRecord")
  {
    ros_type_name = "ros_gz_interfaces/msg/VideoRecord";
    return true;
  }
  if (gz_type_name == "gz.msgs.Clock" || gz_type_name == "ignition.msgs.Clock")
  {
    ros_type_name = "rosgraph_msgs/msg/Clock";
    return true;
  }
  if (gz_type_name == "gz.msgs.BatteryState" || gz_type_name == "ignition.msgs.BatteryState")
  {
    ros_type_name = "sensor_msgs/msg/BatteryState";
    return true;
  }
  if (gz_type_name == "gz.msgs.CameraInfo" || gz_type_name == "ignition.msgs.CameraInfo")
  {
    ros_type_name = "sensor_msgs/msg/CameraInfo";
    return true;
  }
  if (gz_type_name == "gz.msgs.FluidPressure" || gz_type_name == "ignition.msgs.FluidPressure")
  {
    ros_type_name = "sensor_msgs/msg/FluidPressure";
    return true;
  }
  if (gz_type_name == "gz.msgs.Image" || gz_type_name == "ignition.msgs.Image")
  {
    ros_type_name = "sensor_msgs/msg/Image";
    return true;
  }
  if (gz_type_name == "gz.msgs.IMU" || gz_type_name == "ignition.msgs.IMU")
  {
    ros_type_name = "sensor_msgs/msg/Imu";
    return true;
  }
  if (gz_type_name == "gz.msgs.Model" || gz_type_name == "ignition.msgs.Model")
  {
    ros_type_name = "sensor_msgs/msg/JointState";
    return true;
  }
  if (gz_type_name == "gz.msgs.Joy" || gz_type_name == "ignition.msgs.Joy")
  {
    ros_type_name = "sensor_msgs/msg/Joy";
    return true;
  }
  if (gz_type_name == "gz.msgs.LaserScan" || gz_type_name == "ignition.msgs.LaserScan")
  {
    ros_type_name = "sensor_msgs/msg/LaserScan";
    return true;
  }
  if (gz_type_name == "gz.msgs.Magnetometer" || gz_type_name == "ignition.msgs.Magnetometer")
  {
    ros_type_name = "sensor_msgs/msg/MagneticField";
    return true;
  }
  if (gz_type_name == "gz.msgs.NavSat" || gz_type_name == "ignition.msgs.NavSat")
  {
    ros_type_name = "sensor_msgs/msg/NavSatFix";
    return true;
  }
  if (gz_type_name == "gz.msgs.PointCloudPacked" || gz_type_name == "ignition.msgs.PointCloudPacked")
  {
    ros_type_name = "sensor_msgs/msg/PointCloud2";
    return true;
  }
  if (gz_type_name == "gz.msgs.Boolean" || gz_type_name == "ignition.msgs.Boolean")
  {
    ros_type_name = "std_msgs/msg/Bool";
    return true;
  }
  if (gz_type_name == "gz.msgs.Color" || gz_type_name == "ignition.msgs.Color")
  {
    ros_type_name = "std_msgs/msg/ColorRGBA";
    return true;
  }
  if (gz_type_name == "gz.msgs.Empty" || gz_type_name == "ignition.msgs.Empty")
  {
    ros_type_name = "std_msgs/msg/Empty";
    return true;
  }
  if (gz_type_name == "gz.msgs.Float" || gz_type_name == "ignition.msgs.Float")
  {
    ros_type_name = "std_msgs/msg/Float32";
    return true;
  }
  if (gz_type_name == "gz.msgs.Double" || gz_type_name == "ignition.msgs.Double")
  {
    ros_type_name = "std_msgs/msg/Float64";
    return true;
  }
  if (gz_type_name == "gz.msgs.Header" || gz_type_name == "ignition.msgs.Header")
  {
    ros_type_name = "std_msgs/msg/Header";
    return true;
  }
  if (gz_type_name == "gz.msgs.Int32" || gz_type_name == "ignition.msgs.Int32")
  {
    ros_type_name = "std_msgs/msg/Int32";
    return true;
  }
  if (gz_type_name == "gz.msgs.StringMsg" || gz_type_name == "ignition.msgs.StringMsg")
  {
    ros_type_name = "std_msgs/msg/String";
    return true;
  }
  if (gz_type_name == "gz.msgs.UInt32" || gz_type_name == "ignition.msgs.UInt32")
  {
    ros_type_name = "std_msgs/msg/UInt32";
    return true;
  }
  if (gz_type_name == "gz.msgs.Pose_V" || gz_type_name == "ignition.msgs.Pose_V")
  {
    ros_type_name = "tf2_msgs/msg/TFMessage";
    return true;
  }
  if (gz_type_name == "gz.msgs.JointTrajectory" || gz_type_name == "ignition.msgs.JointTrajectory")
  {
    ros_type_name = "trajectory_msgs/msg/JointTrajectory";
    return true;
  }
  if (gz_type_name == "gz.msgs.AnnotatedAxisAligned2DBox" || gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox")
  {
    ros_type_name = "vision_msgs/msg/Detection2D";
    return true;
  }
  if (gz_type_name == "gz.msgs.AnnotatedAxisAligned2DBox_V" || gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox_V")
  {
    ros_type_name = "vision_msgs/msg/Detection2DArray";
    return true;
  }
  if (gz_type_name == "gz.msgs.AnnotatedOriented3DBox" || gz_type_name == "ignition.msgs.AnnotatedOriented3DBox")
  {
    ros_type_name = "vision_msgs/msg/Detection3D";
    return true;
  }
  if (gz_type_name == "gz.msgs.AnnotatedOriented3DBox_V" || gz_type_name == "ignition.msgs.AnnotatedOriented3DBox_V")
  {
    ros_type_name = "vision_msgs/msg/Detection3DArray";
    return true;
  }

  return false;
}

bool
get_ros_to_gz_mapping(const std::string & ros_type_name, std::string & gz_type_name)
{

  if (ros_type_name == "actuator_msgs/msg/Actuators")
  {
    gz_type_name = "gz.msgs.Actuators";
    return true;
  }
  if (ros_type_name == "builtin_interfaces/msg/Time")
  {
    gz_type_name = "gz.msgs.Time";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Point")
  {
    gz_type_name = "gz.msgs.Vector3d";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Pose")
  {
    gz_type_name = "gz.msgs.Pose";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/PoseArray")
  {
    gz_type_name = "gz.msgs.Pose_V";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/PoseStamped")
  {
    gz_type_name = "gz.msgs.Pose";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/PoseWithCovariance")
  {
    gz_type_name = "gz.msgs.PoseWithCovariance";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Quaternion")
  {
    gz_type_name = "gz.msgs.Quaternion";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Transform")
  {
    gz_type_name = "gz.msgs.Pose";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/TransformStamped")
  {
    gz_type_name = "gz.msgs.Pose";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Twist")
  {
    gz_type_name = "gz.msgs.Twist";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/TwistStamped")
  {
    gz_type_name = "gz.msgs.Twist";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/TwistWithCovariance")
  {
    gz_type_name = "gz.msgs.TwistWithCovariance";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/TwistWithCovarianceStamped")
  {
    gz_type_name = "gz.msgs.TwistWithCovariance";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Vector3")
  {
    gz_type_name = "gz.msgs.Vector3d";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/Wrench")
  {
    gz_type_name = "gz.msgs.Wrench";
    return true;
  }
  if (ros_type_name == "geometry_msgs/msg/WrenchStamped")
  {
    gz_type_name = "gz.msgs.Wrench";
    return true;
  }
  if (ros_type_name == "gps_msgs/msg/GPSFix")
  {
    gz_type_name = "gz.msgs.NavSat";
    return true;
  }
  if (ros_type_name == "nav_msgs/msg/Odometry")
  {
    gz_type_name = "gz.msgs.Odometry";
    return true;
  }
  if (ros_type_name == "nav_msgs/msg/Odometry")
  {
    gz_type_name = "gz.msgs.OdometryWithCovariance";
    return true;
  }
  if (ros_type_name == "rcl_interfaces/msg/ParameterValue")
  {
    gz_type_name = "gz.msgs.Any";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Altimeter")
  {
    gz_type_name = "gz.msgs.Altimeter";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Contact")
  {
    gz_type_name = "gz.msgs.Contact";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Contacts")
  {
    gz_type_name = "gz.msgs.Contacts";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Dataframe")
  {
    gz_type_name = "gz.msgs.Dataframe";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Entity")
  {
    gz_type_name = "gz.msgs.Entity";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/EntityWrench")
  {
    gz_type_name = "gz.msgs.EntityWrench";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Float32Array")
  {
    gz_type_name = "gz.msgs.Float_V";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/GuiCamera")
  {
    gz_type_name = "gz.msgs.GUICamera";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/JointWrench")
  {
    gz_type_name = "gz.msgs.JointWrench";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/Light")
  {
    gz_type_name = "gz.msgs.Light";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/ParamVec")
  {
    gz_type_name = "gz.msgs.Param";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/ParamVec")
  {
    gz_type_name = "gz.msgs.Param_V";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/SensorNoise")
  {
    gz_type_name = "gz.msgs.SensorNoise";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/StringVec")
  {
    gz_type_name = "gz.msgs.StringMsg_V";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/TrackVisual")
  {
    gz_type_name = "gz.msgs.TrackVisual";
    return true;
  }
  if (ros_type_name == "ros_gz_interfaces/msg/VideoRecord")
  {
    gz_type_name = "gz.msgs.VideoRecord";
    return true;
  }
  if (ros_type_name == "rosgraph_msgs/msg/Clock")
  {
    gz_type_name = "gz.msgs.Clock";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/BatteryState")
  {
    gz_type_name = "gz.msgs.BatteryState";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/CameraInfo")
  {
    gz_type_name = "gz.msgs.CameraInfo";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/FluidPressure")
  {
    gz_type_name = "gz.msgs.FluidPressure";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/Image")
  {
    gz_type_name = "gz.msgs.Image";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/Imu")
  {
    gz_type_name = "gz.msgs.IMU";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/JointState")
  {
    gz_type_name = "gz.msgs.Model";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/Joy")
  {
    gz_type_name = "gz.msgs.Joy";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/LaserScan")
  {
    gz_type_name = "gz.msgs.LaserScan";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/MagneticField")
  {
    gz_type_name = "gz.msgs.Magnetometer";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/NavSatFix")
  {
    gz_type_name = "gz.msgs.NavSat";
    return true;
  }
  if (ros_type_name == "sensor_msgs/msg/PointCloud2")
  {
    gz_type_name = "gz.msgs.PointCloudPacked";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Bool")
  {
    gz_type_name = "gz.msgs.Boolean";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/ColorRGBA")
  {
    gz_type_name = "gz.msgs.Color";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Empty")
  {
    gz_type_name = "gz.msgs.Empty";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Float32")
  {
    gz_type_name = "gz.msgs.Float";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Float64")
  {
    gz_type_name = "gz.msgs.Double";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Header")
  {
    gz_type_name = "gz.msgs.Header";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/Int32")
  {
    gz_type_name = "gz.msgs.Int32";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/String")
  {
    gz_type_name = "gz.msgs.StringMsg";
    return true;
  }
  if (ros_type_name == "std_msgs/msg/UInt32")
  {
    gz_type_name = "gz.msgs.UInt32";
    return true;
  }
  if (ros_type_name == "tf2_msgs/msg/TFMessage")
  {
    gz_type_name = "gz.msgs.Pose_V";
    return true;
  }
  if (ros_type_name == "trajectory_msgs/msg/JointTrajectory")
  {
    gz_type_name = "gz.msgs.JointTrajectory";
    return true;
  }
  if (ros_type_name == "vision_msgs/msg/Detection2D")
  {
    gz_type_name = "gz.msgs.AnnotatedAxisAligned2DBox";
    return true;
  }
  if (ros_type_name == "vision_msgs/msg/Detection2DArray")
  {
    gz_type_name = "gz.msgs.AnnotatedAxisAligned2DBox_V";
    return true;
  }
  if (ros_type_name == "vision_msgs/msg/Detection3D")
  {
    gz_type_name = "gz.msgs.AnnotatedOriented3DBox";
    return true;
  }
  if (ros_type_name == "vision_msgs/msg/Detection3DArray")
  {
    gz_type_name = "gz.msgs.AnnotatedOriented3DBox_V";
    return true;
  }

  return false;
}

std::multimap<std::string, std::string>
get_all_message_mappings_ros_to_gz()
{
  static std::multimap<std::string, std::string> mappings = {
    {
      "actuator_msgs/msg/Actuators",  // ROS 2
      "gz.msgs.Actuators", // Gazebo
    },
    {
      "actuator_msgs/msg/Actuators",  // ROS 2
      "ignition.msgs.Actuators", // Gazebo
    },
    {
      "builtin_interfaces/msg/Time",  // ROS 2
      "gz.msgs.Time", // Gazebo
    },
    {
      "builtin_interfaces/msg/Time",  // ROS 2
      "ignition.msgs.Time", // Gazebo
    },
    {
      "geometry_msgs/msg/Point",  // ROS 2
      "gz.msgs.Vector3d", // Gazebo
    },
    {
      "geometry_msgs/msg/Point",  // ROS 2
      "ignition.msgs.Vector3d", // Gazebo
    },
    {
      "geometry_msgs/msg/Pose",  // ROS 2
      "gz.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/Pose",  // ROS 2
      "ignition.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseArray",  // ROS 2
      "gz.msgs.Pose_V", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseArray",  // ROS 2
      "ignition.msgs.Pose_V", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseStamped",  // ROS 2
      "gz.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseStamped",  // ROS 2
      "ignition.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseWithCovariance",  // ROS 2
      "gz.msgs.PoseWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/PoseWithCovariance",  // ROS 2
      "ignition.msgs.PoseWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/Quaternion",  // ROS 2
      "gz.msgs.Quaternion", // Gazebo
    },
    {
      "geometry_msgs/msg/Quaternion",  // ROS 2
      "ignition.msgs.Quaternion", // Gazebo
    },
    {
      "geometry_msgs/msg/Transform",  // ROS 2
      "gz.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/Transform",  // ROS 2
      "ignition.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/TransformStamped",  // ROS 2
      "gz.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/TransformStamped",  // ROS 2
      "ignition.msgs.Pose", // Gazebo
    },
    {
      "geometry_msgs/msg/Twist",  // ROS 2
      "gz.msgs.Twist", // Gazebo
    },
    {
      "geometry_msgs/msg/Twist",  // ROS 2
      "ignition.msgs.Twist", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistStamped",  // ROS 2
      "gz.msgs.Twist", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistStamped",  // ROS 2
      "ignition.msgs.Twist", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistWithCovariance",  // ROS 2
      "gz.msgs.TwistWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistWithCovariance",  // ROS 2
      "ignition.msgs.TwistWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistWithCovarianceStamped",  // ROS 2
      "gz.msgs.TwistWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/TwistWithCovarianceStamped",  // ROS 2
      "ignition.msgs.TwistWithCovariance", // Gazebo
    },
    {
      "geometry_msgs/msg/Vector3",  // ROS 2
      "gz.msgs.Vector3d", // Gazebo
    },
    {
      "geometry_msgs/msg/Vector3",  // ROS 2
      "ignition.msgs.Vector3d", // Gazebo
    },
    {
      "geometry_msgs/msg/Wrench",  // ROS 2
      "gz.msgs.Wrench", // Gazebo
    },
    {
      "geometry_msgs/msg/Wrench",  // ROS 2
      "ignition.msgs.Wrench", // Gazebo
    },
    {
      "geometry_msgs/msg/WrenchStamped",  // ROS 2
      "gz.msgs.Wrench", // Gazebo
    },
    {
      "geometry_msgs/msg/WrenchStamped",  // ROS 2
      "ignition.msgs.Wrench", // Gazebo
    },
    {
      "gps_msgs/msg/GPSFix",  // ROS 2
      "gz.msgs.NavSat", // Gazebo
    },
    {
      "gps_msgs/msg/GPSFix",  // ROS 2
      "ignition.msgs.NavSat", // Gazebo
    },
    {
      "nav_msgs/msg/Odometry",  // ROS 2
      "gz.msgs.Odometry", // Gazebo
    },
    {
      "nav_msgs/msg/Odometry",  // ROS 2
      "ignition.msgs.Odometry", // Gazebo
    },
    {
      "nav_msgs/msg/Odometry",  // ROS 2
      "gz.msgs.OdometryWithCovariance", // Gazebo
    },
    {
      "nav_msgs/msg/Odometry",  // ROS 2
      "ignition.msgs.OdometryWithCovariance", // Gazebo
    },
    {
      "rcl_interfaces/msg/ParameterValue",  // ROS 2
      "gz.msgs.Any", // Gazebo
    },
    {
      "rcl_interfaces/msg/ParameterValue",  // ROS 2
      "ignition.msgs.Any", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Altimeter",  // ROS 2
      "gz.msgs.Altimeter", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Altimeter",  // ROS 2
      "ignition.msgs.Altimeter", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Contact",  // ROS 2
      "gz.msgs.Contact", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Contact",  // ROS 2
      "ignition.msgs.Contact", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Contacts",  // ROS 2
      "gz.msgs.Contacts", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Contacts",  // ROS 2
      "ignition.msgs.Contacts", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Dataframe",  // ROS 2
      "gz.msgs.Dataframe", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Dataframe",  // ROS 2
      "ignition.msgs.Dataframe", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Entity",  // ROS 2
      "gz.msgs.Entity", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Entity",  // ROS 2
      "ignition.msgs.Entity", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/EntityWrench",  // ROS 2
      "gz.msgs.EntityWrench", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/EntityWrench",  // ROS 2
      "ignition.msgs.EntityWrench", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Float32Array",  // ROS 2
      "gz.msgs.Float_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Float32Array",  // ROS 2
      "ignition.msgs.Float_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/GuiCamera",  // ROS 2
      "gz.msgs.GUICamera", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/GuiCamera",  // ROS 2
      "ignition.msgs.GUICamera", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/JointWrench",  // ROS 2
      "gz.msgs.JointWrench", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/JointWrench",  // ROS 2
      "ignition.msgs.JointWrench", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Light",  // ROS 2
      "gz.msgs.Light", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/Light",  // ROS 2
      "ignition.msgs.Light", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/ParamVec",  // ROS 2
      "gz.msgs.Param", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/ParamVec",  // ROS 2
      "ignition.msgs.Param", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/ParamVec",  // ROS 2
      "gz.msgs.Param_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/ParamVec",  // ROS 2
      "ignition.msgs.Param_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/SensorNoise",  // ROS 2
      "gz.msgs.SensorNoise", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/SensorNoise",  // ROS 2
      "ignition.msgs.SensorNoise", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/StringVec",  // ROS 2
      "gz.msgs.StringMsg_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/StringVec",  // ROS 2
      "ignition.msgs.StringMsg_V", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/TrackVisual",  // ROS 2
      "gz.msgs.TrackVisual", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/TrackVisual",  // ROS 2
      "ignition.msgs.TrackVisual", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/VideoRecord",  // ROS 2
      "gz.msgs.VideoRecord", // Gazebo
    },
    {
      "ros_gz_interfaces/msg/VideoRecord",  // ROS 2
      "ignition.msgs.VideoRecord", // Gazebo
    },
    {
      "rosgraph_msgs/msg/Clock",  // ROS 2
      "gz.msgs.Clock", // Gazebo
    },
    {
      "rosgraph_msgs/msg/Clock",  // ROS 2
      "ignition.msgs.Clock", // Gazebo
    },
    {
      "sensor_msgs/msg/BatteryState",  // ROS 2
      "gz.msgs.BatteryState", // Gazebo
    },
    {
      "sensor_msgs/msg/BatteryState",  // ROS 2
      "ignition.msgs.BatteryState", // Gazebo
    },
    {
      "sensor_msgs/msg/CameraInfo",  // ROS 2
      "gz.msgs.CameraInfo", // Gazebo
    },
    {
      "sensor_msgs/msg/CameraInfo",  // ROS 2
      "ignition.msgs.CameraInfo", // Gazebo
    },
    {
      "sensor_msgs/msg/FluidPressure",  // ROS 2
      "gz.msgs.FluidPressure", // Gazebo
    },
    {
      "sensor_msgs/msg/FluidPressure",  // ROS 2
      "ignition.msgs.FluidPressure", // Gazebo
    },
    {
      "sensor_msgs/msg/Image",  // ROS 2
      "gz.msgs.Image", // Gazebo
    },
    {
      "sensor_msgs/msg/Image",  // ROS 2
      "ignition.msgs.Image", // Gazebo
    },
    {
      "sensor_msgs/msg/Imu",  // ROS 2
      "gz.msgs.IMU", // Gazebo
    },
    {
      "sensor_msgs/msg/Imu",  // ROS 2
      "ignition.msgs.IMU", // Gazebo
    },
    {
      "sensor_msgs/msg/JointState",  // ROS 2
      "gz.msgs.Model", // Gazebo
    },
    {
      "sensor_msgs/msg/JointState",  // ROS 2
      "ignition.msgs.Model", // Gazebo
    },
    {
      "sensor_msgs/msg/Joy",  // ROS 2
      "gz.msgs.Joy", // Gazebo
    },
    {
      "sensor_msgs/msg/Joy",  // ROS 2
      "ignition.msgs.Joy", // Gazebo
    },
    {
      "sensor_msgs/msg/LaserScan",  // ROS 2
      "gz.msgs.LaserScan", // Gazebo
    },
    {
      "sensor_msgs/msg/LaserScan",  // ROS 2
      "ignition.msgs.LaserScan", // Gazebo
    },
    {
      "sensor_msgs/msg/MagneticField",  // ROS 2
      "gz.msgs.Magnetometer", // Gazebo
    },
    {
      "sensor_msgs/msg/MagneticField",  // ROS 2
      "ignition.msgs.Magnetometer", // Gazebo
    },
    {
      "sensor_msgs/msg/NavSatFix",  // ROS 2
      "gz.msgs.NavSat", // Gazebo
    },
    {
      "sensor_msgs/msg/NavSatFix",  // ROS 2
      "ignition.msgs.NavSat", // Gazebo
    },
    {
      "sensor_msgs/msg/PointCloud2",  // ROS 2
      "gz.msgs.PointCloudPacked", // Gazebo
    },
    {
      "sensor_msgs/msg/PointCloud2",  // ROS 2
      "ignition.msgs.PointCloudPacked", // Gazebo
    },
    {
      "std_msgs/msg/Bool",  // ROS 2
      "gz.msgs.Boolean", // Gazebo
    },
    {
      "std_msgs/msg/Bool",  // ROS 2
      "ignition.msgs.Boolean", // Gazebo
    },
    {
      "std_msgs/msg/ColorRGBA",  // ROS 2
      "gz.msgs.Color", // Gazebo
    },
    {
      "std_msgs/msg/ColorRGBA",  // ROS 2
      "ignition.msgs.Color", // Gazebo
    },
    {
      "std_msgs/msg/Empty",  // ROS 2
      "gz.msgs.Empty", // Gazebo
    },
    {
      "std_msgs/msg/Empty",  // ROS 2
      "ignition.msgs.Empty", // Gazebo
    },
    {
      "std_msgs/msg/Float32",  // ROS 2
      "gz.msgs.Float", // Gazebo
    },
    {
      "std_msgs/msg/Float32",  // ROS 2
      "ignition.msgs.Float", // Gazebo
    },
    {
      "std_msgs/msg/Float64",  // ROS 2
      "gz.msgs.Double", // Gazebo
    },
    {
      "std_msgs/msg/Float64",  // ROS 2
      "ignition.msgs.Double", // Gazebo
    },
    {
      "std_msgs/msg/Header",  // ROS 2
      "gz.msgs.Header", // Gazebo
    },
    {
      "std_msgs/msg/Header",  // ROS 2
      "ignition.msgs.Header", // Gazebo
    },
    {
      "std_msgs/msg/Int32",  // ROS 2
      "gz.msgs.Int32", // Gazebo
    },
    {
      "std_msgs/msg/Int32",  // ROS 2
      "ignition.msgs.Int32", // Gazebo
    },
    {
      "std_msgs/msg/String",  // ROS 2
      "gz.msgs.StringMsg", // Gazebo
    },
    {
      "std_msgs/msg/String",  // ROS 2
      "ignition.msgs.StringMsg", // Gazebo
    },
    {
      "std_msgs/msg/UInt32",  // ROS 2
      "gz.msgs.UInt32", // Gazebo
    },
    {
      "std_msgs/msg/UInt32",  // ROS 2
      "ignition.msgs.UInt32", // Gazebo
    },
    {
      "tf2_msgs/msg/TFMessage",  // ROS 2
      "gz.msgs.Pose_V", // Gazebo
    },
    {
      "tf2_msgs/msg/TFMessage",  // ROS 2
      "ignition.msgs.Pose_V", // Gazebo
    },
    {
      "trajectory_msgs/msg/JointTrajectory",  // ROS 2
      "gz.msgs.JointTrajectory", // Gazebo
    },
    {
      "trajectory_msgs/msg/JointTrajectory",  // ROS 2
      "ignition.msgs.JointTrajectory", // Gazebo
    },
    {
      "vision_msgs/msg/Detection2D",  // ROS 2
      "gz.msgs.AnnotatedAxisAligned2DBox", // Gazebo
    },
    {
      "vision_msgs/msg/Detection2D",  // ROS 2
      "ignition.msgs.AnnotatedAxisAligned2DBox", // Gazebo
    },
    {
      "vision_msgs/msg/Detection2DArray",  // ROS 2
      "gz.msgs.AnnotatedAxisAligned2DBox_V", // Gazebo
    },
    {
      "vision_msgs/msg/Detection2DArray",  // ROS 2
      "ignition.msgs.AnnotatedAxisAligned2DBox_V", // Gazebo
    },
    {
      "vision_msgs/msg/Detection3D",  // ROS 2
      "gz.msgs.AnnotatedOriented3DBox", // Gazebo
    },
    {
      "vision_msgs/msg/Detection3D",  // ROS 2
      "ignition.msgs.AnnotatedOriented3DBox", // Gazebo
    },
    {
      "vision_msgs/msg/Detection3DArray",  // ROS 2
      "gz.msgs.AnnotatedOriented3DBox_V", // Gazebo
    },
    {
      "vision_msgs/msg/Detection3DArray",  // ROS 2
      "ignition.msgs.AnnotatedOriented3DBox_V", // Gazebo
    },
  };
  return mappings;
}

}  // namespace ros_gz_bridge
