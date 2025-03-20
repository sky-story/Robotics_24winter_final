// generated from ros_gz_bridge/resource/pkg_factories.cpp.em


#include "factories/ros_gz_interfaces.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/ros_gz_interfaces.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__ros_gz_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  if ((ros_type_name == "ros_gz_interfaces/msg/Altimeter" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Altimeter" || gz_type_name == "ignition.msgs.Altimeter"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Altimeter,
        gz::msgs::Altimeter
      >
    >("ros_gz_interfaces/msg/Altimeter", "gz.msgs.Altimeter");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Contact" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Contact" || gz_type_name == "ignition.msgs.Contact"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Contact,
        gz::msgs::Contact
      >
    >("ros_gz_interfaces/msg/Contact", "gz.msgs.Contact");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Contacts" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Contacts" || gz_type_name == "ignition.msgs.Contacts"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Contacts,
        gz::msgs::Contacts
      >
    >("ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Dataframe" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Dataframe" || gz_type_name == "ignition.msgs.Dataframe"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Dataframe,
        gz::msgs::Dataframe
      >
    >("ros_gz_interfaces/msg/Dataframe", "gz.msgs.Dataframe");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Entity" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Entity" || gz_type_name == "ignition.msgs.Entity"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Entity,
        gz::msgs::Entity
      >
    >("ros_gz_interfaces/msg/Entity", "gz.msgs.Entity");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/EntityWrench" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.EntityWrench" || gz_type_name == "ignition.msgs.EntityWrench"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::EntityWrench,
        gz::msgs::EntityWrench
      >
    >("ros_gz_interfaces/msg/EntityWrench", "gz.msgs.EntityWrench");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Float32Array" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Float_V" || gz_type_name == "ignition.msgs.Float_V"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Float32Array,
        gz::msgs::Float_V
      >
    >("ros_gz_interfaces/msg/Float32Array", "gz.msgs.Float_V");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/GuiCamera" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.GUICamera" || gz_type_name == "ignition.msgs.GUICamera"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::GuiCamera,
        gz::msgs::GUICamera
      >
    >("ros_gz_interfaces/msg/GuiCamera", "gz.msgs.GUICamera");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/JointWrench" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.JointWrench" || gz_type_name == "ignition.msgs.JointWrench"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::JointWrench,
        gz::msgs::JointWrench
      >
    >("ros_gz_interfaces/msg/JointWrench", "gz.msgs.JointWrench");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/Light" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Light" || gz_type_name == "ignition.msgs.Light"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::Light,
        gz::msgs::Light
      >
    >("ros_gz_interfaces/msg/Light", "gz.msgs.Light");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/ParamVec" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Param" || gz_type_name == "ignition.msgs.Param"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::ParamVec,
        gz::msgs::Param
      >
    >("ros_gz_interfaces/msg/ParamVec", "gz.msgs.Param");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/ParamVec" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.Param_V" || gz_type_name == "ignition.msgs.Param_V"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::ParamVec,
        gz::msgs::Param_V
      >
    >("ros_gz_interfaces/msg/ParamVec", "gz.msgs.Param_V");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/SensorNoise" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.SensorNoise" || gz_type_name == "ignition.msgs.SensorNoise"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::SensorNoise,
        gz::msgs::SensorNoise
      >
    >("ros_gz_interfaces/msg/SensorNoise", "gz.msgs.SensorNoise");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/StringVec" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.StringMsg_V" || gz_type_name == "ignition.msgs.StringMsg_V"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::StringVec,
        gz::msgs::StringMsg_V
      >
    >("ros_gz_interfaces/msg/StringVec", "gz.msgs.StringMsg_V");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/TrackVisual" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.TrackVisual" || gz_type_name == "ignition.msgs.TrackVisual"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::TrackVisual,
        gz::msgs::TrackVisual
      >
    >("ros_gz_interfaces/msg/TrackVisual", "gz.msgs.TrackVisual");
  }
  if ((ros_type_name == "ros_gz_interfaces/msg/VideoRecord" || ros_type_name.empty()) &&
      (gz_type_name == "gz.msgs.VideoRecord" || gz_type_name == "ignition.msgs.VideoRecord"))
  {
    return std::make_shared<
      Factory<
        ros_gz_interfaces::msg::VideoRecord,
        gz::msgs::VideoRecord
      >
    >("ros_gz_interfaces/msg/VideoRecord", "gz.msgs.VideoRecord");
  }
  return nullptr;
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Altimeter,
  gz::msgs::Altimeter
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Altimeter & ros_msg,
  gz::msgs::Altimeter & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Altimeter,
  gz::msgs::Altimeter
>::convert_gz_to_ros(
  const gz::msgs::Altimeter & gz_msg,
  ros_gz_interfaces::msg::Altimeter & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Contact,
  gz::msgs::Contact
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contact & ros_msg,
  gz::msgs::Contact & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Contact,
  gz::msgs::Contact
>::convert_gz_to_ros(
  const gz::msgs::Contact & gz_msg,
  ros_gz_interfaces::msg::Contact & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Contacts,
  gz::msgs::Contacts
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contacts & ros_msg,
  gz::msgs::Contacts & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Contacts,
  gz::msgs::Contacts
>::convert_gz_to_ros(
  const gz::msgs::Contacts & gz_msg,
  ros_gz_interfaces::msg::Contacts & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Dataframe,
  gz::msgs::Dataframe
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  gz::msgs::Dataframe & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Dataframe,
  gz::msgs::Dataframe
>::convert_gz_to_ros(
  const gz::msgs::Dataframe & gz_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Entity,
  gz::msgs::Entity
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Entity & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Entity,
  gz::msgs::Entity
>::convert_gz_to_ros(
  const gz::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::EntityWrench,
  gz::msgs::EntityWrench
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::EntityWrench & ros_msg,
  gz::msgs::EntityWrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::EntityWrench,
  gz::msgs::EntityWrench
>::convert_gz_to_ros(
  const gz::msgs::EntityWrench & gz_msg,
  ros_gz_interfaces::msg::EntityWrench & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Float32Array,
  gz::msgs::Float_V
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Float32Array & ros_msg,
  gz::msgs::Float_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Float32Array,
  gz::msgs::Float_V
>::convert_gz_to_ros(
  const gz::msgs::Float_V & gz_msg,
  ros_gz_interfaces::msg::Float32Array & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::GuiCamera,
  gz::msgs::GUICamera
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::GuiCamera & ros_msg,
  gz::msgs::GUICamera & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::GuiCamera,
  gz::msgs::GUICamera
>::convert_gz_to_ros(
  const gz::msgs::GUICamera & gz_msg,
  ros_gz_interfaces::msg::GuiCamera & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::JointWrench,
  gz::msgs::JointWrench
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::JointWrench & ros_msg,
  gz::msgs::JointWrench & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::JointWrench,
  gz::msgs::JointWrench
>::convert_gz_to_ros(
  const gz::msgs::JointWrench & gz_msg,
  ros_gz_interfaces::msg::JointWrench & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::Light,
  gz::msgs::Light
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::Light & ros_msg,
  gz::msgs::Light & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::Light,
  gz::msgs::Light
>::convert_gz_to_ros(
  const gz::msgs::Light & gz_msg,
  ros_gz_interfaces::msg::Light & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::ParamVec,
  gz::msgs::Param
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::ParamVec,
  gz::msgs::Param
>::convert_gz_to_ros(
  const gz::msgs::Param & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::ParamVec,
  gz::msgs::Param_V
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::ParamVec,
  gz::msgs::Param_V
>::convert_gz_to_ros(
  const gz::msgs::Param_V & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::SensorNoise,
  gz::msgs::SensorNoise
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::SensorNoise & ros_msg,
  gz::msgs::SensorNoise & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::SensorNoise,
  gz::msgs::SensorNoise
>::convert_gz_to_ros(
  const gz::msgs::SensorNoise & gz_msg,
  ros_gz_interfaces::msg::SensorNoise & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::StringVec,
  gz::msgs::StringMsg_V
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::StringVec & ros_msg,
  gz::msgs::StringMsg_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::StringVec,
  gz::msgs::StringMsg_V
>::convert_gz_to_ros(
  const gz::msgs::StringMsg_V & gz_msg,
  ros_gz_interfaces::msg::StringVec & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::TrackVisual,
  gz::msgs::TrackVisual
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::TrackVisual & ros_msg,
  gz::msgs::TrackVisual & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::TrackVisual,
  gz::msgs::TrackVisual
>::convert_gz_to_ros(
  const gz::msgs::TrackVisual & gz_msg,
  ros_gz_interfaces::msg::TrackVisual & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
template<>
void
Factory<
  ros_gz_interfaces::msg::VideoRecord,
  gz::msgs::VideoRecord
>::convert_ros_to_gz(
  const ros_gz_interfaces::msg::VideoRecord & ros_msg,
  gz::msgs::VideoRecord & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  ros_gz_interfaces::msg::VideoRecord,
  gz::msgs::VideoRecord
>::convert_gz_to_ros(
  const gz::msgs::VideoRecord & gz_msg,
  ros_gz_interfaces::msg::VideoRecord & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
