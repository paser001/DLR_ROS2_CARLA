// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice
#include "carla_parking_msgs/msg/detail/vehicle_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
carla_parking_msgs__msg__VehicleControl__init(carla_parking_msgs__msg__VehicleControl * msg)
{
  if (!msg) {
    return false;
  }
  // throttle
  // steer
  // brake
  // reverse
  // hand_brake
  // manual_gear_shift
  // gear
  return true;
}

void
carla_parking_msgs__msg__VehicleControl__fini(carla_parking_msgs__msg__VehicleControl * msg)
{
  if (!msg) {
    return;
  }
  // throttle
  // steer
  // brake
  // reverse
  // hand_brake
  // manual_gear_shift
  // gear
}

bool
carla_parking_msgs__msg__VehicleControl__are_equal(const carla_parking_msgs__msg__VehicleControl * lhs, const carla_parking_msgs__msg__VehicleControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // steer
  if (lhs->steer != rhs->steer) {
    return false;
  }
  // brake
  if (lhs->brake != rhs->brake) {
    return false;
  }
  // reverse
  if (lhs->reverse != rhs->reverse) {
    return false;
  }
  // hand_brake
  if (lhs->hand_brake != rhs->hand_brake) {
    return false;
  }
  // manual_gear_shift
  if (lhs->manual_gear_shift != rhs->manual_gear_shift) {
    return false;
  }
  // gear
  if (lhs->gear != rhs->gear) {
    return false;
  }
  return true;
}

bool
carla_parking_msgs__msg__VehicleControl__copy(
  const carla_parking_msgs__msg__VehicleControl * input,
  carla_parking_msgs__msg__VehicleControl * output)
{
  if (!input || !output) {
    return false;
  }
  // throttle
  output->throttle = input->throttle;
  // steer
  output->steer = input->steer;
  // brake
  output->brake = input->brake;
  // reverse
  output->reverse = input->reverse;
  // hand_brake
  output->hand_brake = input->hand_brake;
  // manual_gear_shift
  output->manual_gear_shift = input->manual_gear_shift;
  // gear
  output->gear = input->gear;
  return true;
}

carla_parking_msgs__msg__VehicleControl *
carla_parking_msgs__msg__VehicleControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_parking_msgs__msg__VehicleControl * msg = (carla_parking_msgs__msg__VehicleControl *)allocator.allocate(sizeof(carla_parking_msgs__msg__VehicleControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(carla_parking_msgs__msg__VehicleControl));
  bool success = carla_parking_msgs__msg__VehicleControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
carla_parking_msgs__msg__VehicleControl__destroy(carla_parking_msgs__msg__VehicleControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    carla_parking_msgs__msg__VehicleControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
carla_parking_msgs__msg__VehicleControl__Sequence__init(carla_parking_msgs__msg__VehicleControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_parking_msgs__msg__VehicleControl * data = NULL;

  if (size) {
    data = (carla_parking_msgs__msg__VehicleControl *)allocator.zero_allocate(size, sizeof(carla_parking_msgs__msg__VehicleControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = carla_parking_msgs__msg__VehicleControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        carla_parking_msgs__msg__VehicleControl__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
carla_parking_msgs__msg__VehicleControl__Sequence__fini(carla_parking_msgs__msg__VehicleControl__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      carla_parking_msgs__msg__VehicleControl__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

carla_parking_msgs__msg__VehicleControl__Sequence *
carla_parking_msgs__msg__VehicleControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_parking_msgs__msg__VehicleControl__Sequence * array = (carla_parking_msgs__msg__VehicleControl__Sequence *)allocator.allocate(sizeof(carla_parking_msgs__msg__VehicleControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = carla_parking_msgs__msg__VehicleControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
carla_parking_msgs__msg__VehicleControl__Sequence__destroy(carla_parking_msgs__msg__VehicleControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    carla_parking_msgs__msg__VehicleControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
carla_parking_msgs__msg__VehicleControl__Sequence__are_equal(const carla_parking_msgs__msg__VehicleControl__Sequence * lhs, const carla_parking_msgs__msg__VehicleControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!carla_parking_msgs__msg__VehicleControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
carla_parking_msgs__msg__VehicleControl__Sequence__copy(
  const carla_parking_msgs__msg__VehicleControl__Sequence * input,
  carla_parking_msgs__msg__VehicleControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(carla_parking_msgs__msg__VehicleControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    carla_parking_msgs__msg__VehicleControl * data =
      (carla_parking_msgs__msg__VehicleControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!carla_parking_msgs__msg__VehicleControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          carla_parking_msgs__msg__VehicleControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!carla_parking_msgs__msg__VehicleControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
