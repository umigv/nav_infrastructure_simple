import math

from geometry_msgs.msg import Point
from nav_utils.geometry import (
    distance,
    get_yaw_radians_from_quaternion,
    make_pose,
    make_quaternion_from_yaw,
    point_is_close,
    rotate_by_yaw,
)


def test_point_is_close_exact_same():
    p1 = Point(x=1.0, y=2.0, z=3.0)
    p2 = Point(x=1.0, y=2.0, z=3.0)
    assert point_is_close(p1, p2)


def test_point_is_close_within_tolerance():
    p1 = Point(x=1.0, y=2.0, z=3.0)
    p2 = Point(x=1.005, y=1.995, z=3.009)
    assert point_is_close(p1, p2)


def test_point_is_close_outside_tolerance():
    p1 = Point(x=1.0, y=2.0, z=3.0)
    p2 = Point(x=1.02, y=2.0, z=3.0)
    assert not point_is_close(p1, p2)


def test_make_quaternion_from_yaw_zero():
    q = make_quaternion_from_yaw(0.0)
    assert q.x == 0.0
    assert q.y == 0.0
    assert abs(q.z - 0.0) < 1e-12
    assert abs(q.w - 1.0) < 1e-12


def test_make_quaternion_from_yaw_positive():
    yaw = math.pi / 2
    q = make_quaternion_from_yaw(yaw)
    assert abs(q.z - math.sin(yaw / 2.0)) < 1e-12
    assert abs(q.w - math.cos(yaw / 2.0)) < 1e-12


def test_make_quaternion_from_yaw_negative():
    yaw = -math.pi / 3
    q = make_quaternion_from_yaw(yaw)
    assert abs(q.z - math.sin(yaw / 2.0)) < 1e-12
    assert abs(q.w - math.cos(yaw / 2.0)) < 1e-12


def test_make_quaternion_is_unit_length():
    yaw = 1.789
    q = make_quaternion_from_yaw(yaw)
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    assert abs(norm - 1.0) < 1e-12


def test_make_quaternion_round_trip_yaw():
    yaw_true = -2.1
    q = make_quaternion_from_yaw(yaw_true)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - yaw_true) < 1e-6


def test_get_yaw_radians_from_quaternion_zero():
    q = make_quaternion_from_yaw(0.0)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - 0.0) < 1e-6


def test_get_yaw_radians_from_quaternion_positive():
    yaw_true = math.pi / 2
    q = make_quaternion_from_yaw(yaw_true)
    yaw = get_yaw_radians_from_quaternion(q)
    assert abs(yaw - yaw_true) < 1e-6


def test_get_yaw_radians_from_quaternion_negative():
    yaw_true = -math.pi / 4
    q = make_quaternion_from_yaw(yaw_true)
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


def test_make_pose_sets_position_and_zero_z():
    pose = make_pose(1.25, -3.5, 0.0)
    assert abs(pose.position.x - 1.25) < 1e-12
    assert abs(pose.position.y + 3.5) < 1e-12
    assert pose.position.z == 0.0


def test_make_pose_orientation_matches_yaw():
    yaw_true = 1.234
    pose = make_pose(0.0, 0.0, yaw_true)
    yaw = get_yaw_radians_from_quaternion(pose.orientation)
    assert abs(yaw - yaw_true) < 1e-6
