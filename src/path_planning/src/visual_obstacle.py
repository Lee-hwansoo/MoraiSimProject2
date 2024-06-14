#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg  import ObjectStatusList
from visualization_msgs.msg import MarkerArray, Marker
import tf.transformations
import math

class obstacle_pub:
    def __init__(self):
        rospy.init_node('obstacle_pub', anonymous=True)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        self.obstacle_pub = rospy.Publisher('/rviz/obstacle',MarkerArray, queue_size=1)

        self.is_object = False
        self.published_ids = set()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_object:
                Objects = MarkerArray()
                for obstacle in self.object_data.obstacle_list:
                    if obstacle.unique_id not in self.published_ids:
                        heading_rad = math.radians(obstacle.heading)
                        q = tf.transformations.quaternion_from_euler(0, 0, heading_rad)
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = rospy.Time.now()
                        marker.id = obstacle.unique_id
                        marker.type = Marker.CUBE

                        marker.pose.position.x = obstacle.position.x
                        marker.pose.position.y = obstacle.position.y
                        marker.pose.position.z = obstacle.position.z

                        marker.pose.orientation.x = q[0]
                        marker.pose.orientation.y = q[1]
                        marker.pose.orientation.z = q[2]
                        marker.pose.orientation.w = q[3]

                        marker.scale.x = obstacle.size.x
                        marker.scale.y = obstacle.size.y
                        marker.scale.z = 0
                        marker.color.a = 1.0
                        Objects.markers.append(marker)

                        self.published_ids.add(obstacle.unique_id)

                if Objects.markers:
                    self.obstacle_pub.publish(Objects)

            rate.sleep()


    def object_callback(self, msg):
        self.is_object=True
        self.object_data = msg


if __name__ == '__main__':
    try:
        obstacle_pub = obstacle_pub()
    except rospy.ROSInterruptException:
        pass
