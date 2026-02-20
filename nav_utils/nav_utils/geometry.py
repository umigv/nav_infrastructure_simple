import math

from geometry_msgs.msg import Point, Pose, Quaternion


def point_is_close(pointA: Point, pointB: Point) -> bool:
    """Whether two points are close enough for nav's purposes"""
    return (
        math.isclose(pointA.x, pointB.x, abs_tol=0.01)
        and math.isclose(pointA.y, pointB.y, abs_tol=0.01)
        and math.isclose(pointA.z, pointB.z, abs_tol=0.01)
    )


def get_yaw_radians_from_quaternion(q: Quaternion) -> float:
    """Extract radians of yaw rotation from Quaternion https://en.wikipedia.org/wiki/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_quaternion_from_yaw(yaw: float) -> Quaternion:
    """ROS quaternion for yaw about +Z."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def rotate_by_yaw(point: Point, angle: float) -> Point:
    c = math.cos(angle)
    s = math.sin(angle)
    return Point(x=c * point.x - s * point.y, y=s * point.x + c * point.y, z=point.z)


def distance(pointA: Point, pointB: Point) -> float:
    return math.hypot(pointA.x - pointB.x, pointA.y - pointB.y)


def make_pose(x: float, y: float, yaw: float) -> Pose:
    """Make 2d robot pose"""
    p = Pose()
    p.position = Point(x=x, y=y, z=0.0)
    p.orientation = make_quaternion_from_yaw(yaw)
    return p
