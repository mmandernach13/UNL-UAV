#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from uav_interfaces.action import GoToPos
from uav_interfaces.msg import UavPos
import numpy as np


class MissionPlannerClientNode(Node):
    def __init__(self):
        super().__init__("mission_planner_client_node")
        self.mission_planner_client = ActionClient(self, GoToPos, "go_to_position")
        self.get_logger().info("Mission Planner Client Node has been started.")
        
        self.mission_dir = os.getcwd() + "/src/mission/mission_files/"
    
    def send_goal(self, target_pos: UavPos):
        self.mission_planner_client.wait_for_server()
        goal = GoToPos.Goal()
        goal.target_pos = target_pos
        self.mission_planner_client.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback). \
            add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
            global pts, idx
            idx += 1
            if idx < len(pts):
                self.send_goal(pts[idx])
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.message))

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Got feedback: " + str(feedback.distance_remaining))

    def get_waypoints_txt(self, filename: str):
        filename = self.mission_dir + filename
        waypoints = np.loadtxt(filename, delimiter=',') # csv format x, y, z, yaw, type
        return waypoints
        
    def run_mission(self, filename: str = "ex_mission.txt"):
        global pts, idx
        waypoints = self.get_waypoints_txt(filename)
        pts = []
        for wp in waypoints:
            pos = UavPos()
            pos.pos[0] = float(wp[0])
            pos.pos[1] = float(wp[1])
            pos.pos[2] = float(wp[2])
            pos.yaw = float(wp[3])
            pos.type = int(wp[4])
            pts.append(pos)
        idx = 0
        self.send_goal(pts[idx])


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerClientNode()

    node.run_mission()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()