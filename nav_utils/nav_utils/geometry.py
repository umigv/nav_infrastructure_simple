from geometry_msgs.msg import Point, Quaternion
import math

def get_yaw_radians_from_quaternion(q: Quaternion) -> float:
    """Extract radians of yaw rotation from Quaternion https://en.wikipedia.org/wiki/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def rotate_by_yaw(point: Point, angle: float) -> Point:
    c = math.cos(angle)
    s = math.sin(angle)
    return Point(x=c * point.x - s * point.y, 
                 y=s * point.x + c * point.y,
                 z=point.z)

def distance(pointA: Point, pointB: Point) -> float:
    return math.hypot(pointA.x - pointB.x, pointA.y - pointB.y)
