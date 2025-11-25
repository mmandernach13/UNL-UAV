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

    mission_states_str_to_int = {
        "IDLE": MissionState.MISSION_STATE_TYPE_IDLE,
        "MODE_CONTROL": MissionState.MISSION_STATE_TYPE_MODE_CONTROL,
        "OFFBOARD": MissionState.MISSION_STATE_TYPE_OFFBOARD,
        "RTL": MissionState.MISSION_STATE_TYPE_RTL,
        "SCAN": MissionState.MISSION_STATE_TYPE_SCAN,
        "PAYLOAD_FOUND": MissionState.MISSION_STATE_TYPE_PAYLOAD_FOUND,
        "DROP_PAYLOAD": MissionState.MISSION_STATE_TYPE_DROP_PAYLOAD
    }

    mission_states_int_to_str = {v: k for k, v in mission_states_str_to_int.items()}

    pos_types_str_to_int = {
        "TAKEOFF": UavPos.UAV_POS_TYPE_TAKEOFF,
        "WAYPOINT": UavPos.UAV_POS_TYPE_WAYPOINT,
        "LAND": UavPos.UAV_POS_TYPE_LAND,
    }

    pos_types_int_to_str = {v: k for k, v in pos_types_str_to_int.items()}

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

        self.mission_state = MissionState.MISSION_STATE_TYPE_IDLE
        self.mission_state_pub = self.create_publisher(
            MissionState,
            '/mission/state',
            qos_profile
        )
        self.get_logger().info('Mission state pub has been created')
        
        self.mission_dir = os.getcwd() + "/src/mission/mission_files/"
    
    # publish the current mission state
    def publish_mission_state(self) -> None:
        msg = MissionState()
        msg.state = self.mission_state
        self.mission_state_pub.publish(msg)
        self.get_logger().info(f"Published mission state: {self.mission_states_int_to_str.get(self.mission_state)}")

    # send a position goal async
    def send_goal(self, target_pos: UavPos) -> None:
        self.mission_planner_client.wait_for_server()
        goal = GoToPos.Goal()
        goal.target_pos = target_pos
        self.mission_planner_client.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback). \
            add_done_callback(self.goal_response_callback)

    # cancel a goal async
    def cancel_goal(self) -> None:
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()

    # get the response if the goal is accepted 
    def goal_response_callback(self, future) -> None:
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    # get the result once the goal is finished
    # handles success, abort, and cancels
    def goal_result_callback(self, future) -> None:
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
            global idx
            idx += 1
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.message))

    # get the feedback from the action
    def goal_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info("Got feedback: " + str(feedback.distance_remaining))

    # parse the mission file (.csv)
    # csv in format [state, x, y, z, yaw, type]
    # state in MISSION_STATE_TYPE_x: x, y, z in NED frame (z is down) 
    # yaw in (0, 360) deg: type in UAV_POS_TYPE_x
    def read_mission_file(self, filepath: str = "ex_mission.csv") -> list[tuple[int, UavPos]]:
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
                point.type = self.pos_types_str_to_int.get(tmp_type, UavPos.UAV_POS_TYPE_WAYPOINT)
                state = self.mission_states_str_to_int.get(tmp_state, MissionState.MISSION_STATE_TYPE_IDLE)
                mission_points.append((state, point))
        return mission_points
    
    # load and run the mission from the given file
    def run_mission(self, filename: str = "ex_mission.csv") -> None:
        global idx
        idx = 0
        mission_filepath = os.path.join(self.mission_dir, filename)
        mission_points = self.read_mission_file(mission_filepath)
        total_points = len(mission_points)

        itr = 0 

        while rclpy.ok() and idx < total_points:
            if itr == idx:  # itr and idx will be equal first loop, then idx will also be incremented after goal success
                state, point = mission_points[idx]
                self.mission_state = state
                self.publish_mission_state()
                self.get_logger().info(f"Sending goal {idx + 1}/{total_points}: State={self.mission_states_int_to_str.get(state)}, \
                                    Pos=({point.pos[0]}, {point.pos[1]}, {point.pos[2]}), Yaw={point.yaw}, Type={self.pos_types_str_to_int.get(type)}")
                self.send_goal(point)
                itr += 1

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerClientNode()

    node.run_mission()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()