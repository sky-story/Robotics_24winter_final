// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/geometry_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/geometry_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__geometry_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "geometry_msgs/msg/Point" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Vector3d" || gz_type_name == "ignition.msgs.Vector3d"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Point,
        gz::msgs::Vector3d
      >
    >("geometry_msgs/msg/Point", "gz.msgs.Vector3d");
  }
  if ((ros_type_name == "geometry_msgs/msg/Pose" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Pose,
        gz::msgs::Pose
      >
    >("geometry_msgs/msg/Pose", "gz.msgs.Pose");
  }
  if ((ros_type_name == "geometry_msgs/msg/PoseArray" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose_V" || gz_type_name == "ignition.msgs.Pose_V"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::PoseArray,
        gz::msgs::Pose_V
      >
    >("geometry_msgs/msg/PoseArray", "gz.msgs.Pose_V");
  }
  if ((ros_type_name == "geometry_msgs/msg/PoseStamped" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::PoseStamped,
        gz::msgs::Pose
      >
    >("geometry_msgs/msg/PoseStamped", "gz.msgs.Pose");
  }
  if ((ros_type_name == "geometry_msgs/msg/PoseWithCovariance" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.PoseWithCovariance" || gz_type_name == "ignition.msgs.PoseWithCovariance"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::PoseWithCovariance,
        gz::msgs::PoseWithCovariance
      >
    >("geometry_msgs/msg/PoseWithCovariance", "gz.msgs.PoseWithCovariance");
  }
  if ((ros_type_name == "geometry_msgs/msg/Quaternion" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Quaternion" || gz_type_name == "ignition.msgs.Quaternion"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Quaternion,
        gz::msgs::Quaternion
      >
    >("geometry_msgs/msg/Quaternion", "gz.msgs.Quaternion");
  }
  if ((ros_type_name == "geometry_msgs/msg/Transform" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Transform,
        gz::msgs::Pose
      >
    >("geometry_msgs/msg/Transform", "gz.msgs.Pose");
  }
  if ((ros_type_name == "geometry_msgs/msg/TransformStamped" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Pose" || gz_type_name == "ignition.msgs.Pose"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TransformStamped,
        gz::msgs::Pose
      >
    >("geometry_msgs/msg/TransformStamped", "gz.msgs.Pose");
  }
  if ((ros_type_name == "geometry_msgs/msg/Twist" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Twist" || gz_type_name == "ignition.msgs.Twist"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Twist,
        gz::msgs::Twist
      >
    >("geometry_msgs/msg/Twist", "gz.msgs.Twist");
  }
  if ((ros_type_name == "geometry_msgs/msg/TwistStamped" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Twist" || gz_type_name == "ignition.msgs.Twist"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TwistStamped,
        gz::msgs::Twist
      >
    >("geometry_msgs/msg/TwistStamped", "gz.msgs.Twist");
  }
  if ((ros_type_name == "geometry_msgs/msg/TwistWithCovariance" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.TwistWithCovariance" || gz_type_name == "ignition.msgs.TwistWithCovariance"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TwistWithCovariance,
        gz::msgs::TwistWithCovariance
      >
    >("geometry_msgs/msg/TwistWithCovariance", "gz.msgs.TwistWithCovariance");
  }
  if ((ros_type_name == "geometry_msgs/msg/TwistWithCovarianceStamped" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.TwistWithCovariance" || gz_type_name == "ignition.msgs.TwistWithCovariance"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::TwistWithCovarianceStamped,
        gz::msgs::TwistWithCovariance
      >
    >("geometry_msgs/msg/TwistWithCovarianceStamped", "gz.msgs.TwistWithCovariance");
  }
  if ((ros_type_name == "geometry_msgs/msg/Vector3" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Vector3d" || gz_type_name == "ignition.msgs.Vector3d"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Vector3,
        gz::msgs::Vector3d
      >
    >("geometry_msgs/msg/Vector3", "gz.msgs.Vector3d");
  }
  if ((ros_type_name == "geometry_msgs/msg/Wrench" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Wrench" || gz_type_name == "ignition.msgs.Wrench"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::Wrench,
        gz::msgs::Wrench
      >
    >("geometry_msgs/msg/Wrench", "gz.msgs.Wrench");
  }
  if ((ros_type_name == "geometry_msgs/msg/WrenchStamped" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Wrench" || gz_type_name == "ignition.msgs.Wrench"))
  {
    return std::make_shared<
      Factory<
        geometry_msgs::msg::WrenchStamped,
        gz::msgs::Wrench
      >
    >("geometry_msgs/msg/WrenchStamped", "gz.msgs.Wrench");
  }
  return nullptr;
}

template<>
void
Factory<
  geometry_msgs::msg::Point,
  gz::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::msg::Point & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Point,
  gz::msgs::Vector3d
>::convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Point & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Pose,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::Pose & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Pose,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Pose & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::PoseArray,
  gz::msgs::Pose_V
>::convert_ros_to_gz(
  const geometry_msgs::msg::PoseArray & ros_msg,
  gz::msgs::Pose_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseArray,
  gz::msgs::Pose_V
>::convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  geometry_msgs::msg::PoseArray & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::PoseStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseStamped,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::PoseStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::PoseWithCovariance,
  gz::msgs::PoseWithCovariance
>::convert_ros_to_gz(
  const geometry_msgs::msg::PoseWithCovariance & ros_msg,
  gz::msgs::PoseWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::PoseWithCovariance,
  gz::msgs::PoseWithCovariance
>::convert_gz_to_ros(
  const gz::msgs::PoseWithCovariance & gz_msg,
  geometry_msgs::msg::PoseWithCovariance & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  gz::msgs::Quaternion
>::convert_ros_to_gz(
  const geometry_msgs::msg::Quaternion & ros_msg,
  gz::msgs::Quaternion & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Quaternion,
  gz::msgs::Quaternion
>::convert_gz_to_ros(
  const gz::msgs::Quaternion & gz_msg,
  geometry_msgs::msg::Quaternion & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Transform,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::Transform & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Transform,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Transform & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TransformStamped,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::TransformStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Twist,
  gz::msgs::Twist
>::convert_ros_to_gz(
  const geometry_msgs::msg::Twist & ros_msg,
  gz::msgs::Twist & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Twist,
  gz::msgs::Twist
>::convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::Twist & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::TwistStamped,
  gz::msgs::Twist
>::convert_ros_to_gz(
  const geometry_msgs::msg::TwistStamped & ros_msg,
  gz::msgs::Twist & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TwistStamped,
  gz::msgs::Twist
>::convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::TwistStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovariance,
  gz::msgs::TwistWithCovariance
>::convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovariance & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovariance,
  gz::msgs::TwistWithCovariance
>::convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovariance & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovarianceStamped,
  gz::msgs::TwistWithCovariance
>::convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::TwistWithCovarianceStamped,
  gz::msgs::TwistWithCovariance
>::convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  gz::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::msg::Vector3 & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Vector3,
  gz::msgs::Vector3d
>::convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Vector3 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::Wrench,
  gz::msgs::Wrench
>::convert_ros_to_gz(
  const geometry_msgs::msg::Wrench & ros_msg,
  gz::msgs::Wrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::Wrench,
  gz::msgs::Wrench
>::convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::Wrench & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  geometry_msgs::msg::WrenchStamped,
  gz::msgs::Wrench
>::convert_ros_to_gz(
  const geometry_msgs::msg::WrenchStamped & ros_msg,
  gz::msgs::Wrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  geometry_msgs::msg::WrenchStamped,
  gz::msgs::Wrench
>::convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::WrenchStamped & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
