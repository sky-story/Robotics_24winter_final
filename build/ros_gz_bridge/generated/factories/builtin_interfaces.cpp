// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/builtin_interfaces.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/builtin_interfaces.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__builtin_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "builtin_interfaces/msg/Time" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Time" || gz_type_name == "ignition.msgs.Time"))
  {
    return std::make_shared<
      Factory<
        builtin_interfaces::msg::Time,
        gz::msgs::Time
      >
    >("builtin_interfaces/msg/Time", "gz.msgs.Time");
  }
  return nullptr;
}

template<>
void
Factory<
  builtin_interfaces::msg::Time,
  gz::msgs::Time
>::convert_ros_to_gz(
  const builtin_interfaces::msg::Time & ros_msg,
  gz::msgs::Time & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  builtin_interfaces::msg::Time,
  gz::msgs::Time
>::convert_gz_to_ros(
  const gz::msgs::Time & gz_msg,
  builtin_interfaces::msg::Time & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
