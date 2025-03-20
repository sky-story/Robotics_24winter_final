// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/sensor_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/sensor_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__sensor_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "sensor_msgs/msg/BatteryState" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.BatteryState" || gz_type_name == "ignition.msgs.BatteryState"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::BatteryState,
        gz::msgs::BatteryState
      >
    >("sensor_msgs/msg/BatteryState", "gz.msgs.BatteryState");
  }
  if ((ros_type_name == "sensor_msgs/msg/CameraInfo" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.CameraInfo" || gz_type_name == "ignition.msgs.CameraInfo"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::CameraInfo,
        gz::msgs::CameraInfo
      >
    >("sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo");
  }
  if ((ros_type_name == "sensor_msgs/msg/FluidPressure" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.FluidPressure" || gz_type_name == "ignition.msgs.FluidPressure"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::FluidPressure,
        gz::msgs::FluidPressure
      >
    >("sensor_msgs/msg/FluidPressure", "gz.msgs.FluidPressure");
  }
  if ((ros_type_name == "sensor_msgs/msg/Image" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Image" || gz_type_name == "ignition.msgs.Image"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Image,
        gz::msgs::Image
      >
    >("sensor_msgs/msg/Image", "gz.msgs.Image");
  }
  if ((ros_type_name == "sensor_msgs/msg/Imu" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.IMU" || gz_type_name == "ignition.msgs.IMU"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Imu,
        gz::msgs::IMU
      >
    >("sensor_msgs/msg/Imu", "gz.msgs.IMU");
  }
  if ((ros_type_name == "sensor_msgs/msg/JointState" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Model" || gz_type_name == "ignition.msgs.Model"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::JointState,
        gz::msgs::Model
      >
    >("sensor_msgs/msg/JointState", "gz.msgs.Model");
  }
  if ((ros_type_name == "sensor_msgs/msg/Joy" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Joy" || gz_type_name == "ignition.msgs.Joy"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::Joy,
        gz::msgs::Joy
      >
    >("sensor_msgs/msg/Joy", "gz.msgs.Joy");
  }
  if ((ros_type_name == "sensor_msgs/msg/LaserScan" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.LaserScan" || gz_type_name == "ignition.msgs.LaserScan"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::LaserScan,
        gz::msgs::LaserScan
      >
    >("sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan");
  }
  if ((ros_type_name == "sensor_msgs/msg/MagneticField" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Magnetometer" || gz_type_name == "ignition.msgs.Magnetometer"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::MagneticField,
        gz::msgs::Magnetometer
      >
    >("sensor_msgs/msg/MagneticField", "gz.msgs.Magnetometer");
  }
  if ((ros_type_name == "sensor_msgs/msg/NavSatFix" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.NavSat" || gz_type_name == "ignition.msgs.NavSat"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::NavSatFix,
        gz::msgs::NavSat
      >
    >("sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat");
  }
  if ((ros_type_name == "sensor_msgs/msg/PointCloud2" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.PointCloudPacked" || gz_type_name == "ignition.msgs.PointCloudPacked"))
  {
    return std::make_shared<
      Factory<
        sensor_msgs::msg::PointCloud2,
        gz::msgs::PointCloudPacked
      >
    >("sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked");
  }
  return nullptr;
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  gz::msgs::BatteryState
>::convert_ros_to_gz(
  const sensor_msgs::msg::BatteryState & ros_msg,
  gz::msgs::BatteryState & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::BatteryState,
  gz::msgs::BatteryState
>::convert_gz_to_ros(
  const gz::msgs::BatteryState & gz_msg,
  sensor_msgs::msg::BatteryState & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  gz::msgs::CameraInfo
>::convert_ros_to_gz(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  gz::msgs::CameraInfo & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::CameraInfo,
  gz::msgs::CameraInfo
>::convert_gz_to_ros(
  const gz::msgs::CameraInfo & gz_msg,
  sensor_msgs::msg::CameraInfo & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  gz::msgs::FluidPressure
>::convert_ros_to_gz(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  gz::msgs::FluidPressure & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::FluidPressure,
  gz::msgs::FluidPressure
>::convert_gz_to_ros(
  const gz::msgs::FluidPressure & gz_msg,
  sensor_msgs::msg::FluidPressure & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::Image,
  gz::msgs::Image
>::convert_ros_to_gz(
  const sensor_msgs::msg::Image & ros_msg,
  gz::msgs::Image & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Image,
  gz::msgs::Image
>::convert_gz_to_ros(
  const gz::msgs::Image & gz_msg,
  sensor_msgs::msg::Image & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::Imu,
  gz::msgs::IMU
>::convert_ros_to_gz(
  const sensor_msgs::msg::Imu & ros_msg,
  gz::msgs::IMU & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Imu,
  gz::msgs::IMU
>::convert_gz_to_ros(
  const gz::msgs::IMU & gz_msg,
  sensor_msgs::msg::Imu & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::JointState,
  gz::msgs::Model
>::convert_ros_to_gz(
  const sensor_msgs::msg::JointState & ros_msg,
  gz::msgs::Model & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::JointState,
  gz::msgs::Model
>::convert_gz_to_ros(
  const gz::msgs::Model & gz_msg,
  sensor_msgs::msg::JointState & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::Joy,
  gz::msgs::Joy
>::convert_ros_to_gz(
  const sensor_msgs::msg::Joy & ros_msg,
  gz::msgs::Joy & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::Joy,
  gz::msgs::Joy
>::convert_gz_to_ros(
  const gz::msgs::Joy & gz_msg,
  sensor_msgs::msg::Joy & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  gz::msgs::LaserScan
>::convert_ros_to_gz(
  const sensor_msgs::msg::LaserScan & ros_msg,
  gz::msgs::LaserScan & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::LaserScan,
  gz::msgs::LaserScan
>::convert_gz_to_ros(
  const gz::msgs::LaserScan & gz_msg,
  sensor_msgs::msg::LaserScan & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  gz::msgs::Magnetometer
>::convert_ros_to_gz(
  const sensor_msgs::msg::MagneticField & ros_msg,
  gz::msgs::Magnetometer & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::MagneticField,
  gz::msgs::Magnetometer
>::convert_gz_to_ros(
  const gz::msgs::Magnetometer & gz_msg,
  sensor_msgs::msg::MagneticField & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::NavSatFix,
  gz::msgs::NavSat
>::convert_ros_to_gz(
  const sensor_msgs::msg::NavSatFix & ros_msg,
  gz::msgs::NavSat & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::NavSatFix,
  gz::msgs::NavSat
>::convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  sensor_msgs::msg::NavSatFix & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_ros_to_gz(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  gz::msgs::PointCloudPacked & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  sensor_msgs::msg::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
