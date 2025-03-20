// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/rcl_interfaces.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/rcl_interfaces.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__rcl_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "rcl_interfaces/msg/ParameterValue" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Any" || gz_type_name == "ignition.msgs.Any"))
  {
    return std::make_shared<
      Factory<
        rcl_interfaces::msg::ParameterValue,
        gz::msgs::Any
      >
    >("rcl_interfaces/msg/ParameterValue", "gz.msgs.Any");
  }
  return nullptr;
}

template<>
void
Factory<
  rcl_interfaces::msg::ParameterValue,
  gz::msgs::Any
>::convert_ros_to_gz(
  const rcl_interfaces::msg::ParameterValue & ros_msg,
  gz::msgs::Any & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  rcl_interfaces::msg::ParameterValue,
  gz::msgs::Any
>::convert_gz_to_ros(
  const gz::msgs::Any & gz_msg,
  rcl_interfaces::msg::ParameterValue & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
