#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

UNKNOWN = -1
FREE = 0

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map = None
        self.map_info = None
        self.exploring = False

    def map_callback(self, msg):
        if self.exploring:
            return
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers found")
            return
        goal = self.frontier_to_goal(frontiers[0])
        self.send_goal(goal)
        self.exploring = True

    def detect_frontiers(self):
        frontiers = []
        h, w = self.map.shape
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if self.map[y, x] == FREE:
                    neighbors = self.map[y-1:y+2, x-1:x+2]
                    if UNKNOWN in neighbors:
                        frontiers.append((x, y))
        return frontiers

    def frontier_to_goal(self, cell):
        x, y = cell
        wx = x * self.map_info.resolution + self.map_info.origin.position.x
        wy = y * self.map_info.resolution + self.map_info.origin.position.y
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.orientation.w = 1.0
        return goal

    def send_goal(self, goal_pose):
        self.get_logger().info("Sending frontier goal")
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.exploring = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Goal reached")
        self.exploring = False

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
