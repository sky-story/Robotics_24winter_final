// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/rosgraph_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/rosgraph_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__rosgraph_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "rosgraph_msgs/msg/Clock" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Clock" || gz_type_name == "ignition.msgs.Clock"))
  {
    return std::make_shared<
      Factory<
        rosgraph_msgs::msg::Clock,
        gz::msgs::Clock
      >
    >("rosgraph_msgs/msg/Clock", "gz.msgs.Clock");
  }
  return nullptr;
}

template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  gz::msgs::Clock
>::convert_ros_to_gz(
  const rosgraph_msgs::msg::Clock & ros_msg,
  gz::msgs::Clock & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  rosgraph_msgs::msg::Clock,
  gz::msgs::Clock
>::convert_gz_to_ros(
  const gz::msgs::Clock & gz_msg,
  rosgraph_msgs::msg::Clock & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
