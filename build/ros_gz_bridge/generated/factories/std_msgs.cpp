// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/std_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/std_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__std_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "std_msgs/msg/Bool" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Boolean" || gz_type_name == "ignition.msgs.Boolean"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Bool,
        gz::msgs::Boolean
      >
    >("std_msgs/msg/Bool", "gz.msgs.Boolean");
  }
  if ((ros_type_name == "std_msgs/msg/ColorRGBA" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Color" || gz_type_name == "ignition.msgs.Color"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::ColorRGBA,
        gz::msgs::Color
      >
    >("std_msgs/msg/ColorRGBA", "gz.msgs.Color");
  }
  if ((ros_type_name == "std_msgs/msg/Empty" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Empty" || gz_type_name == "ignition.msgs.Empty"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Empty,
        gz::msgs::Empty
      >
    >("std_msgs/msg/Empty", "gz.msgs.Empty");
  }
  if ((ros_type_name == "std_msgs/msg/Float32" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Float" || gz_type_name == "ignition.msgs.Float"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Float32,
        gz::msgs::Float
      >
    >("std_msgs/msg/Float32", "gz.msgs.Float");
  }
  if ((ros_type_name == "std_msgs/msg/Float64" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Double" || gz_type_name == "ignition.msgs.Double"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Float64,
        gz::msgs::Double
      >
    >("std_msgs/msg/Float64", "gz.msgs.Double");
  }
  if ((ros_type_name == "std_msgs/msg/Header" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Header" || gz_type_name == "ignition.msgs.Header"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Header,
        gz::msgs::Header
      >
    >("std_msgs/msg/Header", "gz.msgs.Header");
  }
  if ((ros_type_name == "std_msgs/msg/Int32" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Int32" || gz_type_name == "ignition.msgs.Int32"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::Int32,
        gz::msgs::Int32
      >
    >("std_msgs/msg/Int32", "gz.msgs.Int32");
  }
  if ((ros_type_name == "std_msgs/msg/String" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.StringMsg" || gz_type_name == "ignition.msgs.StringMsg"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::String,
        gz::msgs::StringMsg
      >
    >("std_msgs/msg/String", "gz.msgs.StringMsg");
  }
  if ((ros_type_name == "std_msgs/msg/UInt32" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.UInt32" || gz_type_name == "ignition.msgs.UInt32"))
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::UInt32,
        gz::msgs::UInt32
      >
    >("std_msgs/msg/UInt32", "gz.msgs.UInt32");
  }
  return nullptr;
}

template<>
void
Factory<
  std_msgs::msg::Bool,
  gz::msgs::Boolean
>::convert_ros_to_gz(
  const std_msgs::msg::Bool & ros_msg,
  gz::msgs::Boolean & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Bool,
  gz::msgs::Boolean
>::convert_gz_to_ros(
  const gz::msgs::Boolean & gz_msg,
  std_msgs::msg::Bool & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::ColorRGBA,
  gz::msgs::Color
>::convert_ros_to_gz(
  const std_msgs::msg::ColorRGBA & ros_msg,
  gz::msgs::Color & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::ColorRGBA,
  gz::msgs::Color
>::convert_gz_to_ros(
  const gz::msgs::Color & gz_msg,
  std_msgs::msg::ColorRGBA & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::Empty,
  gz::msgs::Empty
>::convert_ros_to_gz(
  const std_msgs::msg::Empty & ros_msg,
  gz::msgs::Empty & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Empty,
  gz::msgs::Empty
>::convert_gz_to_ros(
  const gz::msgs::Empty & gz_msg,
  std_msgs::msg::Empty & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::Float32,
  gz::msgs::Float
>::convert_ros_to_gz(
  const std_msgs::msg::Float32 & ros_msg,
  gz::msgs::Float & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float32,
  gz::msgs::Float
>::convert_gz_to_ros(
  const gz::msgs::Float & gz_msg,
  std_msgs::msg::Float32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::Float64,
  gz::msgs::Double
>::convert_ros_to_gz(
  const std_msgs::msg::Float64 & ros_msg,
  gz::msgs::Double & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Float64,
  gz::msgs::Double
>::convert_gz_to_ros(
  const gz::msgs::Double & gz_msg,
  std_msgs::msg::Float64 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::Header,
  gz::msgs::Header
>::convert_ros_to_gz(
  const std_msgs::msg::Header & ros_msg,
  gz::msgs::Header & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Header,
  gz::msgs::Header
>::convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::Int32,
  gz::msgs::Int32
>::convert_ros_to_gz(
  const std_msgs::msg::Int32 & ros_msg,
  gz::msgs::Int32 & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::Int32,
  gz::msgs::Int32
>::convert_gz_to_ros(
  const gz::msgs::Int32 & gz_msg,
  std_msgs::msg::Int32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::String,
  gz::msgs::StringMsg
>::convert_ros_to_gz(
  const std_msgs::msg::String & ros_msg,
  gz::msgs::StringMsg & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::String,
  gz::msgs::StringMsg
>::convert_gz_to_ros(
  const gz::msgs::StringMsg & gz_msg,
  std_msgs::msg::String & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  std_msgs::msg::UInt32,
  gz::msgs::UInt32
>::convert_ros_to_gz(
  const std_msgs::msg::UInt32 & ros_msg,
  gz::msgs::UInt32 & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  std_msgs::msg::UInt32,
  gz::msgs::UInt32
>::convert_gz_to_ros(
  const gz::msgs::UInt32 & gz_msg,
  std_msgs::msg::UInt32 & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
