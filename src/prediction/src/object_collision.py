#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from prediction.msg import TrackedPoint, PredictedObjectPath, PredictedObjectPathList, TrackedObjectPose, TrackedObjectPoseList
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Bool


def check_intersection(p1, p2, p3, p4):
    """
    Check if line segments (p1, p2) and (p3, p4) intersect.
    Returns True if they intersect.
    """
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)


def segment_intersection(p1, p2, p3, p4):
    """
    Find the intersection point of line segments (p1, p2) and (p3, p4)
    if they intersect.
    """
    def det(a, b): return a[0] * b[1] - a[1] * b[0]
    xdiff = (p1[0] - p2[0], p3[0] - p4[0])
    ydiff = (p1[1] - p2[1], p3[1] - p4[1])
    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = (det(p1, p2), det(p3, p4))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return (x, y)


def find_intersections(points1, points2):
    """
    Check for intersections between two lists of points that define two lines.
    Returns a list of intersection points.
    """
    intersections = []
    for i in range(len(points1) - 1):
        for j in range(len(points2) - 1):
            if check_intersection(points1[i], points1[i+1], points2[j], points2[j+1]):
                intersect = segment_intersection(
                    points1[i], points1[i+1], points2[j], points2[j+1])
                if intersect:
                    intersections.append((intersect, (i, j)))

    return intersections

def expand_endpoints(points, width):
    """
    Expand the endpoints of the path to form rectangles based on the given width.
    Returns a rectangle defined by 4 corner points.
    """
    half_width = width / 2.0
    p1 = np.array(points[0])
    p2 = np.array(points[-1])
    direction = p2 - p1
    length = np.linalg.norm(direction)
    unit_direction = direction / length if length != 0 else np.zeros_like(direction)
    normal = np.array([-unit_direction[1], unit_direction[0]])

    rect = [
        tuple(p1 + half_width * normal),
        tuple(p1 - half_width * normal),
        tuple(p2 - half_width * normal),
        tuple(p2 + half_width * normal)
    ]

    return rect

def check_rectangle_intersection(rect1, rect2):
    """
    Check if two rectangles intersect.
    Each rectangle is defined by 4 points in counter-clockwise order.
    """
    for rect in [rect1, rect2]:
        for i in range(4):
            A, B = rect[i], rect[(i + 1) % 4]
            axis = np.array([B[1] - A[1], A[0] - B[0]])  # Perpendicular axis

            projections1 = [np.dot(axis, point) for point in rect1]
            projections2 = [np.dot(axis, point) for point in rect2]

            if max(projections1) < min(projections2) or max(projections2) < min(projections1):
                return False

    return True

class CollisionDetectionNode:
    def __init__(self):
        rospy.init_node('collision_detection_node', anonymous=True)
        rospy.Subscriber('/local_path', Path, self.local_path_callback)
        rospy.Subscriber('/Object_topic/tracked_object_path_topic', PredictedObjectPathList, self.object_path_callback)
        self.pred_collision = rospy.Publisher('/pred_collision', Bool, queue_size=1)

        self.rate = rospy.Rate(10)
        self.is_local_path_received = False
        self.local_path = None
        self.is_object_path_received = False
        self.object_path = None

    def local_path_callback(self, msg):
        self.is_local_path_received=True
        self.local_path = msg

    def object_path_callback(self, msg):
        self.is_object_path_received=True
        self.object_path = msg

    def pub_collision(self):
        if not self.is_local_path_received or not self.is_object_path_received:
            return

        local_path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.local_path.poses]
        if len(local_path_points) < 2:
            return  # Not enough points to form a path

        local_rect = expand_endpoints(local_path_points, 1.87)

        for obstacle in self.object_path.path_list:
            object_path_points = [(pose.x, pose.y) for pose in obstacle.path]
            if len(object_path_points) < 2:
                continue  # Not enough points to form a path

            object_rect = expand_endpoints(object_path_points, 1.87)

            if check_rectangle_intersection(local_rect, object_rect):
                print("Collision detected!")
                self.pred_collision.publish(Bool(data=True))    # Collision detected
                return

        self.pred_collision.publish(Bool(data=False))   # No collision detected

    def run(self):
        while not rospy.is_shutdown():
            if self.is_local_path_received and self.is_object_path_received:
                self.pub_collision()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        collision = CollisionDetectionNode()
        collision.run()
    except rospy.ROSInterruptException:
        pass
