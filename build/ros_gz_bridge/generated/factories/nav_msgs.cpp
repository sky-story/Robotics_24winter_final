// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/nav_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/nav_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__nav_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "nav_msgs/msg/Odometry" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Odometry" || gz_type_name == "ignition.msgs.Odometry"))
  {
    return std::make_shared<
      Factory<
        nav_msgs::msg::Odometry,
        gz::msgs::Odometry
      >
    >("nav_msgs/msg/Odometry", "gz.msgs.Odometry");
  }
  if ((ros_type_name == "nav_msgs/msg/Odometry" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.OdometryWithCovariance" || gz_type_name == "ignition.msgs.OdometryWithCovariance"))
  {
    return std::make_shared<
      Factory<
        nav_msgs::msg::Odometry,
        gz::msgs::OdometryWithCovariance
      >
    >("nav_msgs/msg/Odometry", "gz.msgs.OdometryWithCovariance");
  }
  return nullptr;
}

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  gz::msgs::Odometry
>::convert_ros_to_gz(
  const nav_msgs::msg::Odometry & ros_msg,
  gz::msgs::Odometry & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  gz::msgs::Odometry
>::convert_gz_to_ros(
  const gz::msgs::Odometry & gz_msg,
  nav_msgs::msg::Odometry & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  nav_msgs::msg::Odometry,
  gz::msgs::OdometryWithCovariance
>::convert_ros_to_gz(
  const nav_msgs::msg::Odometry & ros_msg,
  gz::msgs::OdometryWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  nav_msgs::msg::Odometry,
  gz::msgs::OdometryWithCovariance
>::convert_gz_to_ros(
  const gz::msgs::OdometryWithCovariance & gz_msg,
  nav_msgs::msg::Odometry & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
