# generated from rosidl_generator_py/resource/_idl.py.em
# with input from aruco_interface:srv/SendImage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SendImage_Request(type):
    """Metaclass of message 'SendImage_Request'."""

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
            module = import_type_support('aruco_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'aruco_interface.srv.SendImage_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__send_image__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__send_image__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__send_image__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__send_image__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__send_image__request

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SendImage_Request(metaclass=Metaclass_SendImage_Request):
    """Message class 'SendImage_Request'."""

    __slots__ = [
        '_image_data',
    ]

    _fields_and_field_types = {
        'image_data': 'sensor_msgs/Image',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.image_data = kwargs.get('image_data', Image())

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
        if self.image_data != other.image_data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def image_data(self):
        """Message field 'image_data'."""
        return self._image_data

    @image_data.setter
    def image_data(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'image_data' field must be a sub message of type 'Image'"
        self._image_data = value


# Import statements for member types

# Member 'ids'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SendImage_Response(type):
    """Metaclass of message 'SendImage_Response'."""

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
            module = import_type_support('aruco_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'aruco_interface.srv.SendImage_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__send_image__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__send_image__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__send_image__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__send_image__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__send_image__response

            from aruco_interface.msg import BoundingBox
            if BoundingBox.__class__._TYPE_SUPPORT is None:
                BoundingBox.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SendImage_Response(metaclass=Metaclass_SendImage_Response):
    """Message class 'SendImage_Response'."""

    __slots__ = [
        '_ids',
        '_bboxs',
    ]

    _fields_and_field_types = {
        'ids': 'sequence<int32>',
        'bboxs': 'sequence<aruco_interface/BoundingBox>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['aruco_interface', 'msg'], 'BoundingBox')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ids = array.array('i', kwargs.get('ids', []))
        self.bboxs = kwargs.get('bboxs', [])

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
        if self.ids != other.ids:
            return False
        if self.bboxs != other.bboxs:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ids(self):
        """Message field 'ids'."""
        return self._ids

    @ids.setter
    def ids(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'ids' array.array() must have the type code of 'i'"
            self._ids = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'ids' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._ids = array.array('i', value)

    @builtins.property
    def bboxs(self):
        """Message field 'bboxs'."""
        return self._bboxs

    @bboxs.setter
    def bboxs(self, value):
        if __debug__:
            from aruco_interface.msg import BoundingBox
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, BoundingBox) for v in value) and
                 True), \
                "The 'bboxs' field must be a set or sequence and each value of type 'BoundingBox'"
        self._bboxs = value


class Metaclass_SendImage(type):
    """Metaclass of service 'SendImage'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('aruco_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'aruco_interface.srv.SendImage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__send_image

            from aruco_interface.srv import _send_image
            if _send_image.Metaclass_SendImage_Request._TYPE_SUPPORT is None:
                _send_image.Metaclass_SendImage_Request.__import_type_support__()
            if _send_image.Metaclass_SendImage_Response._TYPE_SUPPORT is None:
                _send_image.Metaclass_SendImage_Response.__import_type_support__()


class SendImage(metaclass=Metaclass_SendImage):
    from aruco_interface.srv._send_image import SendImage_Request as Request
    from aruco_interface.srv._send_image import SendImage_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
