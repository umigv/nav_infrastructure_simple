from dataclasses import MISSING, fields, is_dataclass
from typing import Type, TypeVar, Any, get_type_hints
from rclpy.node import Node
import sys

T = TypeVar("T")

# This is vibe coded
def load(node: Node, cls: Type[T], prefix: str = "") -> T:
    kwargs: dict[str, Any] = {}
    
    try:
        frame = sys._getframe(1)
        type_hints = get_type_hints(cls, globalns=frame.f_globals, localns=frame.f_locals)
    except (NameError, AttributeError, ValueError):
        try:
            type_hints = get_type_hints(cls)
        except:
            type_hints = {}
    
    for f in fields(cls):
        key = f"{prefix}{f.name}" if prefix else f.name
        
        field_type = type_hints.get(f.name, f.type)
    
        try:
            is_nested_dataclass = is_dataclass(field_type)
        except (TypeError, AttributeError):
            is_nested_dataclass = False
        
        if is_nested_dataclass:
            kwargs[f.name] = load(node, field_type, prefix=f"{key}.")
            continue
        
        has_default = (f.default is not MISSING)
        has_factory = (getattr(f, "default_factory", MISSING) is not MISSING)
        
        if not has_default and not has_factory:
            node.declare_parameter(key)
            value = node.get_parameter(key).value
            if value is None:
                raise RuntimeError(
                    f"Required parameter '{key}' not set for node '{node.get_name()}'"
                )
            kwargs[f.name] = value
        else:
            if has_default:
                default_value = f.default
            else:
                default_value = f.default_factory()
            node.declare_parameter(key, default_value)
            kwargs[f.name] = node.get_parameter(key).value
    
    return cls(**kwargs)
