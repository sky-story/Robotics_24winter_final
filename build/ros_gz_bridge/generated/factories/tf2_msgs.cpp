// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/tf2_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/tf2_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__tf2_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "tf2_msgs/msg/TFMessage" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose_V" || gz_type_name == "ignition.msgs.Pose_V"))
  {
    return std::make_shared<
      Factory<
        tf2_msgs::msg::TFMessage,
        gz::msgs::Pose_V
      >
    >("tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V");
  }
  return nullptr;
}

template<>
void
Factory<
  tf2_msgs::msg::TFMessage,
  gz::msgs::Pose_V
>::convert_ros_to_gz(
  const tf2_msgs::msg::TFMessage & ros_msg,
  gz::msgs::Pose_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  tf2_msgs::msg::TFMessage,
  gz::msgs::Pose_V
>::convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  tf2_msgs::msg::TFMessage & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
