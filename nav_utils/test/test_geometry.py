import math
from geometry_msgs.msg import Point, Quaternion
from nav_utils.geometry import get_yaw_radians_from_quaternion, rotate_by_yaw, distance

def quat_from_yaw(yaw: float) -> Quaternion:
    """Helper to construct a pure-yaw quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def test_get_yaw_radians_from_quaternion_zero():
    q = quat_from_yaw(0.0)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - 0.0) < 1e-6

def test_get_yaw_radians_from_quaternion_positive():
    yaw_true = math.pi / 2
    q = quat_from_yaw(yaw_true)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - yaw_true) < 1e-6

def test_get_yaw_radians_from_quaternion_negative():
    yaw_true = -math.pi / 4
    q = quat_from_yaw(yaw_true)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - yaw_true) < 1e-6

def test_rotate_zero_angle():
    p = Point(x=1.0, y=2.0, z=0.0)
    r = rotate_by_yaw(p, 0.0)

    assert abs(r.x - 1.0) < 1e-6
    assert abs(r.y - 2.0) < 1e-6
    assert r.z == 0.0

def test_rotate_90_degrees():
    p = Point(x=1.0, y=0.0, z=0.0)
    r = rotate_by_yaw(p, math.pi / 2)

    assert abs(r.x - 0.0) < 1e-6
    assert abs(r.y - 1.0) < 1e-6

def test_rotate_preserves_distance_from_origin():
    p = Point(x=3.0, y=4.0, z=0.0)
    r = rotate_by_yaw(p, 1.2345)

    d1 = math.sqrt(p.x**2 + p.y**2)
    d2 = math.sqrt(r.x**2 + r.y**2)

    assert abs(d1 - d2) < 1e-6

def test_distance_same_point():
    p = Point(x=1.0, y=2.0, z=0.0)
    assert distance(p, p) == 0.0

def test_distance_simple():
    p1 = Point(x=0.0, y=0.0, z=0.0)
    p2 = Point(x=3.0, y=4.0, z=0.0)

    assert abs(distance(p1, p2) - 5.0) < 1e-6

def test_distance_symmetric():
    p1 = Point(x=1.0, y=2.0, z=0.0)
    p2 = Point(x=-3.0, y=4.0, z=0.0)

    assert abs(distance(p1, p2) - distance(p2, p1)) < 1e-6
