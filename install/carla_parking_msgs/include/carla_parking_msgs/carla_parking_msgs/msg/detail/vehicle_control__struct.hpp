// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from carla_parking_msgs:msg/VehicleControl.idl
// generated code does not contain a copyright notice

#ifndef CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_HPP_
#define CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__carla_parking_msgs__msg__VehicleControl __attribute__((deprecated))
#else
# define DEPRECATED__carla_parking_msgs__msg__VehicleControl __declspec(deprecated)
#endif

namespace carla_parking_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleControl_
{
  using Type = VehicleControl_<ContainerAllocator>;

  explicit VehicleControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->brake = 0.0f;
      this->reverse = false;
      this->hand_brake = false;
      this->manual_gear_shift = false;
      this->gear = 0l;
    }
  }

  explicit VehicleControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->brake = 0.0f;
      this->reverse = false;
      this->hand_brake = false;
      this->manual_gear_shift = false;
      this->gear = 0l;
    }
  }

  // field types and members
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _steer_type =
    float;
  _steer_type steer;
  using _brake_type =
    float;
  _brake_type brake;
  using _reverse_type =
    bool;
  _reverse_type reverse;
  using _hand_brake_type =
    bool;
  _hand_brake_type hand_brake;
  using _manual_gear_shift_type =
    bool;
  _manual_gear_shift_type manual_gear_shift;
  using _gear_type =
    int32_t;
  _gear_type gear;

  // setters for named parameter idiom
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__steer(
    const float & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__brake(
    const float & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__reverse(
    const bool & _arg)
  {
    this->reverse = _arg;
    return *this;
  }
  Type & set__hand_brake(
    const bool & _arg)
  {
    this->hand_brake = _arg;
    return *this;
  }
  Type & set__manual_gear_shift(
    const bool & _arg)
  {
    this->manual_gear_shift = _arg;
    return *this;
  }
  Type & set__gear(
    const int32_t & _arg)
  {
    this->gear = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carla_parking_msgs__msg__VehicleControl
    std::shared_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carla_parking_msgs__msg__VehicleControl
    std::shared_ptr<carla_parking_msgs::msg::VehicleControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleControl_ & other) const
  {
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->reverse != other.reverse) {
      return false;
    }
    if (this->hand_brake != other.hand_brake) {
      return false;
    }
    if (this->manual_gear_shift != other.manual_gear_shift) {
      return false;
    }
    if (this->gear != other.gear) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleControl_

// alias to use template instance with default allocator
using VehicleControl =
  carla_parking_msgs::msg::VehicleControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace carla_parking_msgs

#endif  // CARLA_PARKING_MSGS__MSG__DETAIL__VEHICLE_CONTROL__STRUCT_HPP_
