# generated from rosidl_generator_py/resource/_idl.py.em
# with input from carla_parking_msgs:msg/VehicleControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VehicleControl(type):
    """Metaclass of message 'VehicleControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('carla_parking_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'carla_parking_msgs.msg.VehicleControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vehicle_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vehicle_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vehicle_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vehicle_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vehicle_control

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VehicleControl(metaclass=Metaclass_VehicleControl):
    """Message class 'VehicleControl'."""

    __slots__ = [
        '_throttle',
        '_steer',
        '_brake',
        '_reverse',
        '_hand_brake',
        '_manual_gear_shift',
        '_gear',
    ]

    _fields_and_field_types = {
        'throttle': 'float',
        'steer': 'float',
        'brake': 'float',
        'reverse': 'boolean',
        'hand_brake': 'boolean',
        'manual_gear_shift': 'boolean',
        'gear': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.throttle = kwargs.get('throttle', float())
        self.steer = kwargs.get('steer', float())
        self.brake = kwargs.get('brake', float())
        self.reverse = kwargs.get('reverse', bool())
        self.hand_brake = kwargs.get('hand_brake', bool())
        self.manual_gear_shift = kwargs.get('manual_gear_shift', bool())
        self.gear = kwargs.get('gear', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.throttle != other.throttle:
            return False
        if self.steer != other.steer:
            return False
        if self.brake != other.brake:
            return False
        if self.reverse != other.reverse:
            return False
        if self.hand_brake != other.hand_brake:
            return False
        if self.manual_gear_shift != other.manual_gear_shift:
            return False
        if self.gear != other.gear:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def throttle(self):
        """Message field 'throttle'."""
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'throttle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._throttle = value

    @builtins.property
    def steer(self):
        """Message field 'steer'."""
        return self._steer

    @steer.setter
    def steer(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steer' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'steer' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._steer = value

    @builtins.property
    def brake(self):
        """Message field 'brake'."""
        return self._brake

    @brake.setter
    def brake(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'brake' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'brake' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._brake = value

    @builtins.property
    def reverse(self):
        """Message field 'reverse'."""
        return self._reverse

    @reverse.setter
    def reverse(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reverse' field must be of type 'bool'"
        self._reverse = value

    @builtins.property
    def hand_brake(self):
        """Message field 'hand_brake'."""
        return self._hand_brake

    @hand_brake.setter
    def hand_brake(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'hand_brake' field must be of type 'bool'"
        self._hand_brake = value

    @builtins.property
    def manual_gear_shift(self):
        """Message field 'manual_gear_shift'."""
        return self._manual_gear_shift

    @manual_gear_shift.setter
    def manual_gear_shift(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'manual_gear_shift' field must be of type 'bool'"
        self._manual_gear_shift = value

    @builtins.property
    def gear(self):
        """Message field 'gear'."""
        return self._gear

    @gear.setter
    def gear(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gear' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'gear' field must be an integer in [-2147483648, 2147483647]"
        self._gear = value
