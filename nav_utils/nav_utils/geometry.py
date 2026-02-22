import math

from geometry_msgs.msg import Point, Pose, Quaternion


def point_is_close(pointA: Point, pointB: Point) -> bool:
    """Check whether two points are close enough for nav's purposes.

    Args:
        pointA: First point.
        pointB: Second point.

    Returns:
        True if all axes are within 1cm of each other.
    """
    return (
        math.isclose(pointA.x, pointB.x, abs_tol=0.01)
        and math.isclose(pointA.y, pointB.y, abs_tol=0.01)
        and math.isclose(pointA.z, pointB.z, abs_tol=0.01)
    )


def get_yaw_radians_from_quaternion(q: Quaternion) -> float:
    """Extract the yaw angle in radians from a quaternion.

    Args:
        q: ROS quaternion message.

    Returns:
        Yaw rotation in radians about the +Z axis.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_quaternion_from_yaw(yaw: float) -> Quaternion:
    """Construct a ROS quaternion representing a pure yaw rotation about +Z.

    Args:
        yaw: Rotation angle in radians about the +Z axis.

    Returns:
        Quaternion with x=0, y=0, z=sin(yaw/2), w=cos(yaw/2).
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def rotate_by_yaw(point: Point, angle: float) -> Point:
    """Rotate a point about the +Z axis by the given angle.

    Applies a 2D rotation matrix to the x and y components, leaving z unchanged.

    Args:
        point: Point to rotate.
        angle: Rotation angle in radians.

    Returns:
        Rotated point with the same z value.
    """
    c = math.cos(angle)
    s = math.sin(angle)
    return Point(x=c * point.x - s * point.y, y=s * point.x + c * point.y, z=point.z)


def distance(pointA: Point, pointB: Point) -> float:
    """Compute the 2D Euclidean distance between two points.

    The z component is ignored.

    Args:
        pointA: First point.
        pointB: Second point.

    Returns:
        Distance in the XY plane between the two points.
    """
    return math.hypot(pointA.x - pointB.x, pointA.y - pointB.y)


def make_pose(x: float, y: float, yaw: float) -> Pose:
    """Construct a 2D robot pose from position and heading.

    Args:
        x: X position in meters.
        y: Y position in meters.
        yaw: Heading in radians about the +Z axis.

    Returns:
        Pose with z=0 and orientation set from yaw.
    """
    p = Pose()
    p.position = Point(x=x, y=y, z=0.0)
    p.orientation = make_quaternion_from_yaw(yaw)
    return p
