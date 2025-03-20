// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/gps_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/gps_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__gps_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "gps_msgs/msg/GPSFix" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.NavSat" || gz_type_name == "ignition.msgs.NavSat"))
  {
    return std::make_shared<
      Factory<
        gps_msgs::msg::GPSFix,
        gz::msgs::NavSat
      >
    >("gps_msgs/msg/GPSFix", "gz.msgs.NavSat");
  }
  return nullptr;
}

template<>
void
Factory<
  gps_msgs::msg::GPSFix,
  gz::msgs::NavSat
>::convert_ros_to_gz(
  const gps_msgs::msg::GPSFix & ros_msg,
  gz::msgs::NavSat & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  gps_msgs::msg::GPSFix,
  gz::msgs::NavSat
>::convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  gps_msgs::msg::GPSFix & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
