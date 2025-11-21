#!/usr/bin/env python3

import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from uav_interfaces.action import GoToPos
from uav_interfaces.msg import UavPos, MissionState
import numpy as np


class MissionPlannerClientNode(Node):

    mission_states = {
        "IDLE": MissionState.MISSION_STATE_CMD_TYPE_IDLE,
        "MODE_CONTROL": MissionState.MISSION_STATE_CMD_TYPE_MODE_CONTROL,
        "OFFBOARD": MissionState.MISSION_STATE_CMD_TYPE_OFFBOARD,
        "RTL": MissionState.MISSION_STATE_CMD_TYPE_RTL,
        "SCAN": MissionState.MISSION_STATE_CMD_TYPE_SCAN,
        "PAYLOAD_FOUND": MissionState.MISSION_STATE_CMD_TYPE_PAYLOAD_FOUND,
        "DROP_PAYLOAD": MissionState.MISSION_STATE_CMD_TYPE_DROP_PAYLOAD
    }

    pos_types = {
        "TAKEOFF": UavPos.UAV_POS_TYPE_TAKEOFF,
        "WAYPOINT": UavPos.UAV_POS_TYPE_WAYPOINT,
        "LAND": UavPos.UAV_POS_TYPE_LAND,
    }

    def __init__(self):
        super().__init__("mission_planner_client_node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.mission_planner_client = ActionClient(self, GoToPos, "go_to_position")
        self.get_logger().info("Mission Planner Client Node has been started.")

        self.mission_state = MissionState.MISSION_STATE_CMD_TYPE_IDLE
        self.mission_state_pub = self.create_publisher(
            MissionState,
            '/mission/state',
            qos_profile
        )
        
        self.mission_dir = os.getcwd() + "/src/mission/mission_files/"
    
    def publish_mission_state(self):
        msg = MissionState()
        msg.state = self.mission_state
        self.mission_state_pub.publish(msg)

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

    def read_mission_file(self, filepath: str = "ex_mission.csv"):
        mission_points = []
        with open(filepath, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                point = UavPos()
                tmp_state = str(row[0])
                tmp_type = str(row[5])
                point.pos[0] = float(row[1])
                point.pos[1] = float(row[2])
                point.pos[2] = float(row[3])
                point.yaw = float(row[4])
                point.type = self.pos_types.get(tmp_type, UavPos.UAV_POS_TYPE_WAYPOINT)
                state = self.mission_states.get(tmp_state, MissionState.MISSION_STATE_TYPE_IDLE)
                mission_points.append((state, point))
        return mission_points
        
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