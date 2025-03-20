// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/actuator_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/actuator_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__actuator_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "actuator_msgs/msg/Actuators" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Actuators" || gz_type_name == "ignition.msgs.Actuators"))
  {
    return std::make_shared<
      Factory<
        actuator_msgs::msg::Actuators,
        gz::msgs::Actuators
      >
    >("actuator_msgs/msg/Actuators", "gz.msgs.Actuators");
  }
  return nullptr;
}

template<>
void
Factory<
  actuator_msgs::msg::Actuators,
  gz::msgs::Actuators
>::convert_ros_to_gz(
  const actuator_msgs::msg::Actuators & ros_msg,
  gz::msgs::Actuators & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  actuator_msgs::msg::Actuators,
  gz::msgs::Actuators
>::convert_gz_to_ros(
  const gz::msgs::Actuators & gz_msg,
  actuator_msgs::msg::Actuators & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
