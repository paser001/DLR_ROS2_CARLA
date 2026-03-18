// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice

#ifndef CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_H_
#define CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/VehicleControl in the package carla_parking_msgs.
typedef struct carla_parking_msgs__msg__VehicleControl
{
  float throttle;
  float steer;
  float brake;
  bool reverse;
  bool hand_brake;
  bool manual_gear_shift;
  int32_t gear;
} carla_parking_msgs__msg__VehicleControl;

// Struct for a sequence of carla_parking_msgs__msg__VehicleControl.
typedef struct carla_parking_msgs__msg__VehicleControl__Sequence
{
  carla_parking_msgs__msg__VehicleControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_parking_msgs__msg__VehicleControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_H_
