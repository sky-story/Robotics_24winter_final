// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/vision_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/vision_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__vision_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "vision_msgs/msg/Detection2D" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.AnnotatedAxisAligned2DBox" || gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection2D,
        gz::msgs::AnnotatedAxisAligned2DBox
      >
    >("vision_msgs/msg/Detection2D", "gz.msgs.AnnotatedAxisAligned2DBox");
  }
  if ((ros_type_name == "vision_msgs/msg/Detection2DArray" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.AnnotatedAxisAligned2DBox_V" || gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox_V"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection2DArray,
        gz::msgs::AnnotatedAxisAligned2DBox_V
      >
    >("vision_msgs/msg/Detection2DArray", "gz.msgs.AnnotatedAxisAligned2DBox_V");
  }
  if ((ros_type_name == "vision_msgs/msg/Detection3D" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.AnnotatedOriented3DBox" || gz_type_name == "ignition.msgs.AnnotatedOriented3DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection3D,
        gz::msgs::AnnotatedOriented3DBox
      >
    >("vision_msgs/msg/Detection3D", "gz.msgs.AnnotatedOriented3DBox");
  }
  if ((ros_type_name == "vision_msgs/msg/Detection3DArray" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.AnnotatedOriented3DBox_V" || gz_type_name == "ignition.msgs.AnnotatedOriented3DBox_V"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection3DArray,
        gz::msgs::AnnotatedOriented3DBox_V
      >
    >("vision_msgs/msg/Detection3DArray", "gz.msgs.AnnotatedOriented3DBox_V");
  }
  return nullptr;
}

template<>
void
Factory<
  vision_msgs::msg::Detection2D,
  gz::msgs::AnnotatedAxisAligned2DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection2D & ros_msg,
  gz::msgs::AnnotatedAxisAligned2DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2D,
  gz::msgs::AnnotatedAxisAligned2DBox
>::convert_gz_to_ros(
  const gz::msgs::AnnotatedAxisAligned2DBox & gz_msg,
  vision_msgs::msg::Detection2D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  vision_msgs::msg::Detection2DArray,
  gz::msgs::AnnotatedAxisAligned2DBox_V
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection2DArray & ros_msg,
  gz::msgs::AnnotatedAxisAligned2DBox_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2DArray,
  gz::msgs::AnnotatedAxisAligned2DBox_V
>::convert_gz_to_ros(
  const gz::msgs::AnnotatedAxisAligned2DBox_V & gz_msg,
  vision_msgs::msg::Detection2DArray & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  vision_msgs::msg::Detection3D,
  gz::msgs::AnnotatedOriented3DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection3D & ros_msg,
  gz::msgs::AnnotatedOriented3DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3D,
  gz::msgs::AnnotatedOriented3DBox
>::convert_gz_to_ros(
  const gz::msgs::AnnotatedOriented3DBox & gz_msg,
  vision_msgs::msg::Detection3D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  vision_msgs::msg::Detection3DArray,
  gz::msgs::AnnotatedOriented3DBox_V
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection3DArray & ros_msg,
  gz::msgs::AnnotatedOriented3DBox_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3DArray,
  gz::msgs::AnnotatedOriented3DBox_V
>::convert_gz_to_ros(
  const gz::msgs::AnnotatedOriented3DBox_V & gz_msg,
  vision_msgs::msg::Detection3DArray & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
