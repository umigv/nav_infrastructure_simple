"""
nav_utils.config

Dataclass-backed ROS 2 parameter loading.

`load(node, cls, prefix="")` constructs a dataclass instance by reading ROS 2 parameters
from `node`.

Mapping rules:
- Each dataclass field name corresponds to a ROS 2 parameter key.
- If the field has a default value (or default_factory), the parameter is optional and
  the default is used when the key is not supplied in YAML.
- If the field has no default, the parameter is required; `load()` will raise if it
  is missing / unset.
- Nested dataclasses are supported and map to nested parameter dictionaries.

For example, you can create the following config dataclass:
```py
from dataclasses import dataclass
from rclpy.node import Node
import nav_utils.config

@dataclass
class Weights:
    heading: float = 1.0
    clearance: float  # required

@dataclass
class PlannerConfig:
    max_iters: int = 10_000
    timeout_s: float = 0.2
    weights: Weights
```

Which would map to a ROS2 config yaml structure like this:
```yaml
planner:
  ros__parameters:
    # max_iters is optional (defaults to 10000)
    timeout_s: 3.0            # required
    weights:
      # heading is optional (defaults to 1.0)
      clearance: 0.75         # required
```

You can then load it in code by calling nav_utils.config.load
```py
class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.config = nav_utils.config.load(self, PlannerConfig)
        print(self.config.max_iters)
        print(self.config.timeout_s)
        print(self.config.weights.clearance)
        print(self.config.weights.heading)
```
"""
from dataclasses import MISSING, fields, is_dataclass
from typing import Type, TypeVar, Any, get_type_hints
from rclpy.node import Node
import sys

T = TypeVar("T")

# This is vibe coded
def load(node: Node, cls: Type[T], prefix: str = "") -> T:
    """
    Load ROS 2 parameters from `node` into a dataclass instance of type `cls`.

    Args:
        node: ROS 2 node providing parameters.
        cls: Dataclass type to construct.
        prefix: Optional prefix for parameter keys. If non-empty, keys become
            f"{prefix}{field_name}". (Callers typically pass "planner." or similar.)

    Returns:
        An instance of `cls` populated from ROS 2 parameters.

    Raises:
        RuntimeError: If a required parameter (field without a default) is missing or unset.
    """
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
