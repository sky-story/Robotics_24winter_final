// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/trajectory_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/trajectory_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__trajectory_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "trajectory_msgs/msg/JointTrajectory" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.JointTrajectory" || gz_type_name == "ignition.msgs.JointTrajectory"))
  {
    return std::make_shared<
      Factory<
        trajectory_msgs::msg::JointTrajectory,
        gz::msgs::JointTrajectory
      >
    >("trajectory_msgs/msg/JointTrajectory", "gz.msgs.JointTrajectory");
  }
  return nullptr;
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectory,
  gz::msgs::JointTrajectory
>::convert_ros_to_gz(
  const trajectory_msgs::msg::JointTrajectory & ros_msg,
  gz::msgs::JointTrajectory & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  trajectory_msgs::msg::JointTrajectory,
  gz::msgs::JointTrajectory
>::convert_gz_to_ros(
  const gz::msgs::JointTrajectory & gz_msg,
  trajectory_msgs::msg::JointTrajectory & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
