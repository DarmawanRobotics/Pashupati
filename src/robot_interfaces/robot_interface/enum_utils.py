"""Turn any rosidl message class's int constants into a Python Enum."""

import inspect
from enum import Enum


def generateEnumDict(messageType):
    """Collect a message class's int constant members into a name->value dict."""
    enumDict = {}
    for member in inspect.getmembers(messageType):
        if isinstance(member, tuple) and len(member) == 2:
            if isinstance(member[1], int):
                enumDict[member[0]] = member[1]
    return enumDict


def generateEnum(messageType, enumName=None):
    """Build a Python Enum directly from a message class's int constants."""
    return Enum(enumName or messageType.__name__ + "Enum", generateEnumDict(messageType))
