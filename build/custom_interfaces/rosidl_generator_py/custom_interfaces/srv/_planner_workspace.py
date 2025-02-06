# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:srv/PlannerWorkspace.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlannerWorkspace_Request(type):
    """Metaclass of message 'PlannerWorkspace_Request'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.PlannerWorkspace_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__planner_workspace__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__planner_workspace__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__planner_workspace__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__planner_workspace__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__planner_workspace__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlannerWorkspace_Request(metaclass=Metaclass_PlannerWorkspace_Request):
    """Message class 'PlannerWorkspace_Request'."""

    __slots__ = [
        '_upper',
        '_lower',
    ]

    _fields_and_field_types = {
        'upper': 'float',
        'lower': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.upper = kwargs.get('upper', float())
        self.lower = kwargs.get('lower', float())

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
        if self.upper != other.upper:
            return False
        if self.lower != other.lower:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def upper(self):
        """Message field 'upper'."""
        return self._upper

    @upper.setter
    def upper(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'upper' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'upper' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._upper = value

    @builtins.property
    def lower(self):
        """Message field 'lower'."""
        return self._lower

    @lower.setter
    def lower(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lower' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lower' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lower = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PlannerWorkspace_Response(type):
    """Metaclass of message 'PlannerWorkspace_Response'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.PlannerWorkspace_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__planner_workspace__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__planner_workspace__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__planner_workspace__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__planner_workspace__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__planner_workspace__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlannerWorkspace_Response(metaclass=Metaclass_PlannerWorkspace_Response):
    """Message class 'PlannerWorkspace_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_PlannerWorkspace(type):
    """Metaclass of service 'PlannerWorkspace'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.PlannerWorkspace')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__planner_workspace

            from custom_interfaces.srv import _planner_workspace
            if _planner_workspace.Metaclass_PlannerWorkspace_Request._TYPE_SUPPORT is None:
                _planner_workspace.Metaclass_PlannerWorkspace_Request.__import_type_support__()
            if _planner_workspace.Metaclass_PlannerWorkspace_Response._TYPE_SUPPORT is None:
                _planner_workspace.Metaclass_PlannerWorkspace_Response.__import_type_support__()


class PlannerWorkspace(metaclass=Metaclass_PlannerWorkspace):
    from custom_interfaces.srv._planner_workspace import PlannerWorkspace_Request as Request
    from custom_interfaces.srv._planner_workspace import PlannerWorkspace_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
