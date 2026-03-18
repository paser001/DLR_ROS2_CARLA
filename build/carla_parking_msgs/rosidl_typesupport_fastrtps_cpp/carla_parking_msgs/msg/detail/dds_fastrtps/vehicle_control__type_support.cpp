// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice
#include "carla_parking_msgs/msg/detail/vehicle_control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "carla_parking_msgs/msg/detail/vehicle_control__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace carla_parking_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_parking_msgs
cdr_serialize(
  const carla_parking_msgs::msg::VehicleControl & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: throttle
  cdr << ros_message.throttle;
  // Member: steer
  cdr << ros_message.steer;
  // Member: brake
  cdr << ros_message.brake;
  // Member: reverse
  cdr << (ros_message.reverse ? true : false);
  // Member: hand_brake
  cdr << (ros_message.hand_brake ? true : false);
  // Member: manual_gear_shift
  cdr << (ros_message.manual_gear_shift ? true : false);
  // Member: gear
  cdr << ros_message.gear;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_parking_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  carla_parking_msgs::msg::VehicleControl & ros_message)
{
  // Member: throttle
  cdr >> ros_message.throttle;

  // Member: steer
  cdr >> ros_message.steer;

  // Member: brake
  cdr >> ros_message.brake;

  // Member: reverse
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reverse = tmp ? true : false;
  }

  // Member: hand_brake
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.hand_brake = tmp ? true : false;
  }

  // Member: manual_gear_shift
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.manual_gear_shift = tmp ? true : false;
  }

  // Member: gear
  cdr >> ros_message.gear;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_parking_msgs
get_serialized_size(
  const carla_parking_msgs::msg::VehicleControl & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: throttle
  {
    size_t item_size = sizeof(ros_message.throttle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: steer
  {
    size_t item_size = sizeof(ros_message.steer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: brake
  {
    size_t item_size = sizeof(ros_message.brake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reverse
  {
    size_t item_size = sizeof(ros_message.reverse);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hand_brake
  {
    size_t item_size = sizeof(ros_message.hand_brake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: manual_gear_shift
  {
    size_t item_size = sizeof(ros_message.manual_gear_shift);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gear
  {
    size_t item_size = sizeof(ros_message.gear);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_parking_msgs
max_serialized_size_VehicleControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: throttle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: steer
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: brake
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: reverse
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: hand_brake
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: manual_gear_shift
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gear
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_parking_msgs::msg::VehicleControl;
    is_plain =
      (
      offsetof(DataType, gear) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _VehicleControl__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const carla_parking_msgs::msg::VehicleControl *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VehicleControl__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<carla_parking_msgs::msg::VehicleControl *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VehicleControl__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const carla_parking_msgs::msg::VehicleControl *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VehicleControl__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_VehicleControl(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _VehicleControl__callbacks = {
  "carla_parking_msgs::msg",
  "VehicleControl",
  _VehicleControl__cdr_serialize,
  _VehicleControl__cdr_deserialize,
  _VehicleControl__get_serialized_size,
  _VehicleControl__max_serialized_size
};

static rosidl_message_type_support_t _VehicleControl__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VehicleControl__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace carla_parking_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_carla_parking_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<carla_parking_msgs::msg::VehicleControl>()
{
  return &carla_parking_msgs::msg::typesupport_fastrtps_cpp::_VehicleControl__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, carla_parking_msgs, msg, VehicleControl)() {
  return &carla_parking_msgs::msg::typesupport_fastrtps_cpp::_VehicleControl__handle;
}

#ifdef __cplusplus
}
#endif
