#!/usr/bin/env python3
# coding: utf-8

import rospy
import ast
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid


class StartGoalViz:
    def __init__(self):
        rospy.init_node("start_goal_viz", anonymous=False)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.wait_for_message("/map", OccupancyGrid)

        self.frame = rospy.get_param("~frame", "map")

        self.start = self.parse_list_param("~start", "[0,0]")
        self.target = self.parse_list_param("~target", "[5,5]")

        self.pub = rospy.Publisher(
            "start_goal_markers",
            Marker,
            queue_size=1,
            latch=True
        )

        rospy.sleep(0.5)
        
        while not rospy.is_shutdown():
            self.publish_markers()

    def map_cb(self,data):
        self.map = data  

    def parse_list_param(self, name, default):
        raw = rospy.get_param(name, default)

        if isinstance(raw, (list, tuple)):
            return raw

        try:
            value = ast.literal_eval(raw)
            if isinstance(value, (list, tuple)) and len(value) == 2:
                return [float(value[0]), float(value[1])]
        except Exception:
            pass

        rospy.logerr("Invalid format for %s. Expected '[x, y]'", name)
        rospy.signal_shutdown("Invalid parameter")

    def make_marker(self, mid, x, y, r, g, b):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()

        m.ns = "start_goal"
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = (x*self.map.info.resolution + self.map.info.origin.position.x)  + self.map.info.resolution/2  # centro da célula
        m.pose.position.y = (y*self.map.info.resolution + self.map.info.origin.position.y)  + self.map.info.resolution/2
        m.pose.position.z = 0.05

        m.pose.orientation.w = 1.0

        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 0.1

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0

        return m

    def publish_markers(self):
        self.pub.publish(self.make_marker(0, *self.start, 0,1,0))
        self.pub.publish(self.make_marker(1, *self.target, 1,0,0))

if __name__ == "__main__":
    StartGoalViz()
    rospy.spin()
