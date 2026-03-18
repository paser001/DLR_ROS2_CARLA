// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice

#ifndef CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__FUNCTIONS_H_
#define CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "carla_parking_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "carla_parking_msgs/msg/detail/vehicle_control__struct.h"

/// Initialize msg/VehicleControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * carla_parking_msgs__msg__VehicleControl
 * )) before or use
 * carla_parking_msgs__msg__VehicleControl__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__init(carla_parking_msgs__msg__VehicleControl * msg);

/// Finalize msg/VehicleControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
void
carla_parking_msgs__msg__VehicleControl__fini(carla_parking_msgs__msg__VehicleControl * msg);

/// Create msg/VehicleControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * carla_parking_msgs__msg__VehicleControl__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
carla_parking_msgs__msg__VehicleControl *
carla_parking_msgs__msg__VehicleControl__create();

/// Destroy msg/VehicleControl message.
/**
 * It calls
 * carla_parking_msgs__msg__VehicleControl__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
void
carla_parking_msgs__msg__VehicleControl__destroy(carla_parking_msgs__msg__VehicleControl * msg);

/// Check for msg/VehicleControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__are_equal(const carla_parking_msgs__msg__VehicleControl * lhs, const carla_parking_msgs__msg__VehicleControl * rhs);

/// Copy a msg/VehicleControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__copy(
  const carla_parking_msgs__msg__VehicleControl * input,
  carla_parking_msgs__msg__VehicleControl * output);

/// Initialize array of msg/VehicleControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * carla_parking_msgs__msg__VehicleControl__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__Sequence__init(carla_parking_msgs__msg__VehicleControl__Sequence * array, size_t size);

/// Finalize array of msg/VehicleControl messages.
/**
 * It calls
 * carla_parking_msgs__msg__VehicleControl__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
void
carla_parking_msgs__msg__VehicleControl__Sequence__fini(carla_parking_msgs__msg__VehicleControl__Sequence * array);

/// Create array of msg/VehicleControl messages.
/**
 * It allocates the memory for the array and calls
 * carla_parking_msgs__msg__VehicleControl__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
carla_parking_msgs__msg__VehicleControl__Sequence *
carla_parking_msgs__msg__VehicleControl__Sequence__create(size_t size);

/// Destroy array of msg/VehicleControl messages.
/**
 * It calls
 * carla_parking_msgs__msg__VehicleControl__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
void
carla_parking_msgs__msg__VehicleControl__Sequence__destroy(carla_parking_msgs__msg__VehicleControl__Sequence * array);

/// Check for msg/VehicleControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__Sequence__are_equal(const carla_parking_msgs__msg__VehicleControl__Sequence * lhs, const carla_parking_msgs__msg__VehicleControl__Sequence * rhs);

/// Copy an array of msg/VehicleControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_carla_parking_msgs
bool
carla_parking_msgs__msg__VehicleControl__Sequence__copy(
  const carla_parking_msgs__msg__VehicleControl__Sequence * input,
  carla_parking_msgs__msg__VehicleControl__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__FUNCTIONS_H_
