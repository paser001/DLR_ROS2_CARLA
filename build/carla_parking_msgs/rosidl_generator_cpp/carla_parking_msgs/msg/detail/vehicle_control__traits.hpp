// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice

#ifndef CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__TRAITS_HPP_
#define CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_parking_msgs/msg/detail/vehicle_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_parking_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: steer
  {
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << ", ";
  }

  // member: brake
  {
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << ", ";
  }

  // member: reverse
  {
    out << "reverse: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse, out);
    out << ", ";
  }

  // member: hand_brake
  {
    out << "hand_brake: ";
    rosidl_generator_traits::value_to_yaml(msg.hand_brake, out);
    out << ", ";
  }

  // member: manual_gear_shift
  {
    out << "manual_gear_shift: ";
    rosidl_generator_traits::value_to_yaml(msg.manual_gear_shift, out);
    out << ", ";
  }

  // member: gear
  {
    out << "gear: ";
    rosidl_generator_traits::value_to_yaml(msg.gear, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: steer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << "\n";
  }

  // member: brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << "\n";
  }

  // member: reverse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reverse: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse, out);
    out << "\n";
  }

  // member: hand_brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hand_brake: ";
    rosidl_generator_traits::value_to_yaml(msg.hand_brake, out);
    out << "\n";
  }

  // member: manual_gear_shift
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manual_gear_shift: ";
    rosidl_generator_traits::value_to_yaml(msg.manual_gear_shift, out);
    out << "\n";
  }

  // member: gear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gear: ";
    rosidl_generator_traits::value_to_yaml(msg.gear, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleControl & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace carla_parking_msgs

namespace rosidl_generator_traits
{

[[deprecated("use carla_parking_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_parking_msgs::msg::VehicleControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_parking_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_parking_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_parking_msgs::msg::VehicleControl & msg)
{
  return carla_parking_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_parking_msgs::msg::VehicleControl>()
{
  return "carla_parking_msgs::msg::VehicleControl";
}

template<>
inline const char * name<carla_parking_msgs::msg::VehicleControl>()
{
  return "carla_parking_msgs/msg/VehicleControl";
}

template<>
struct has_fixed_size<carla_parking_msgs::msg::VehicleControl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_parking_msgs::msg::VehicleControl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_parking_msgs::msg::VehicleControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__TRAITS_HPP_
