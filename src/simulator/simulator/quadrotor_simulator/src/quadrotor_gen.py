#! /usr/bin/env python3
import rospy
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import json

class GoalMarker:
    def __init__(self):
        self.start = None
        self.goal = None
        self.marker_id = 0
        self.car_id = 0
        self.marker_array = MarkerArray()

    def goal_callback(self, msg):
        cur_posi = [round(msg.pose.position.x, 2), round(msg.pose.position.y)]
        if self.start is None:
            self.start = cur_posi
            self.add_marker(msg.pose.position, (1, 0, 0, 1))  # 红色标记
        elif self.goal is None:
            self.goal = cur_posi
            self.add_marker(msg.pose.position, (0, 1, 0, 1))  # 绿色标记
            self.save_to_json()
            self.generate_car()
            self.car_id += 1
        else:
            rospy.loginfo("Both start and goal points are already collected.")

    def add_marker(self, position, color):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = color[3]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.position = position
        marker.id = self.marker_id
        self.marker_id += 1
        self.marker_array.markers.append(marker)

    def generate_car(self):
        car = {
            'id': self.car_id,
            'start': self.start,
            'goal': self.goal
        }
        rospy.loginfo("Generated car: {}".format(car))
        self.start = None
        self.goal = None

    def save_to_json(self):
        with open(f'/home/krasjet/Documents/ros1/Car-like-Robotic-swarm/start_goal.json', 'r') as file:
            existing_data = json.load(file)
        existing_data[f'car_{self.car_id}'] = {
            'start': self.start,
            'goal': self.goal
          }
        with open(f'/home/krasjet/Documents/ros1/Car-like-Robotic-swarm/start_goal.json', 'w') as file:
            json.dump(existing_data, file)
        rospy.loginfo(f"Data saved to start_goal.json")

def main():
    rospy.init_node('goal_marker_node')
    goal_marker = GoalMarker()
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_marker.goal_callback)
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if goal_marker.marker_array.markers:
            marker_pub.publish(goal_marker.marker_array)

        rate.sleep()

if __name__ == '__main__':
    main()