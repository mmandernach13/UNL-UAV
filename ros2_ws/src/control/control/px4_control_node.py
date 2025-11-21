#!/usr/bin/env python3

import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleControlMode
from uav_interfaces.msg import UavPos, MissionState
from uav_interfaces.action import GoToPos
import math


class PositionController(Node):
    def __init__(self) -> None:
        super().__init__('position_control')

        # QoS Profile for reliable communication with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- State Variables ---
        self.uav_pos: UavPos = UavPos()
        self.last_pos_cmd: UavPos = UavPos()
        self.mission_state: int = -1
        self.uav_airborn: bool = False
        self.offboard_enable: bool = False
        self.armed: bool = False
        self.action_in_progress: bool = False
        self.offboard_signal_msg: OffboardControlMode = OffboardControlMode()

        # --- Parameters ---
        self.declare_parameter('loop_rate', 10.0)
        self.loop_freq: float = self.get_parameter('loop_rate').value
        self.rate = self.create_rate(self.loop_freq)

        self.declare_parameter('offboard_rate', 5.0)
        self.offboard_rate: float = self.get_parameter('offboard_rate').value

        self.declare_parameter('pos_delta', 0.3)
        self.pos_delta: float = self.get_parameter('pos_delta').value

        # --- Action Server ---
        self.action_server: ActionServer = ActionServer(
            self,
            GoToPos,
            'control/execute_movement',
            goal_callback=self.goal_pos_cb,
            cancel_callback=self.cancel_pos_cb,
            execute_callback=self.execute_pos_cb,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Action server started')

        # --- Timers ---
        self.maintain_offboard_timer = self.create_timer(1 / self.offboard_rate, self.maintain_offboard_cb)
        self.get_logger().info('Offboard maintainer started')

        # --- Subscribers ---
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_cb, qos_profile)
        self.get_logger().info('Position sub created')

        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_cb, qos_profile)
        self.get_logger().info('Control mode sub created')

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_profile)
        self.get_logger().info('Status sub created')

        self.mission_state_sub = self.create_subscription(
            MissionState, '/mission/mission_state', self.mission_state_cb, qos_profile)
        self.get_logger().info('Mission state sub created')

        # --- Publishers ---
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.get_logger().info('Command pub created')

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.get_logger().info('Control mode pub created')

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.get_logger().info('Trajectory pub created')

    def send_offboard_signal(self) -> None:
        self.offboard_signal_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher.publish(self.offboard_signal_msg)

    def enter_offboard_mode(self) -> None:
        self.get_logger().info('Requesting OFFBOARD mode...')
        self.offboard_signal_msg.position = True
        self.offboard_signal_msg.velocity = False
        self.offboard_signal_msg.acceleration = False
        self.offboard_signal_msg.attitude = False
        self.offboard_signal_msg.body_rate = False
        
        # Continuously send offboard signal and request mode switch
        itr: int = 0
        while not self.offboard_enable:
            self.send_offboard_signal()
            if itr > 10:
                cmd_msg: VehicleCommand = VehicleCommand()
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                cmd_msg.param1 = 1.0  # Custom mode
                cmd_msg.param2 = 6.0  # Offboard mode
                cmd_msg.target_system = 1
                cmd_msg.target_component = 1
                cmd_msg.from_external = True
                cmd_msg.timestamp = self.get_clock().now().nanoseconds // 1000
                self.vehicle_command_publisher.publish(cmd_msg)
            itr += 1
            self.rate.sleep()
        self.get_logger().info('OFFBOARD mode enabled')

    def arm_uav(self):
        self.get_logger().info('Arming UAV')
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 1.0 # 1 to arm, 0 to disarm
        
        max_attempts = 100
        attempts = 0
        
        while (self.armed == False) and (attempts < max_attempts):
            self.get_logger().info(f'Attempting to arm... (attempt {attempts})')

            cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(cmd_msg)
           
            attempts += 1
            self.rate.sleep()
        
        if self.armed:
            self.get_logger().info('UAV armed')
        else:
            self.get_logger().error('Failed to arm! Check PX4 console')

    def disarm_uav(self) -> None:
        self.get_logger().info('Disarming UAV...')
        cmd_msg: VehicleCommand = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 0.0  # 0 to disarm
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        
        while self.armed: # Wait until disarmed
            cmd_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.vehicle_command_publisher.publish(cmd_msg)
            self.get_logger().info('Attempting to disarm...')
            self.rate.sleep()
        
        if not self.armed:
            self.get_logger().info('UAV disarmed')
        else:
            self.get_logger().error('Failed to disarm! Check PX4 console')

    def goal_pos_cb(self, goal_request: GoToPos.Goal) -> GoalResponse:
        """Accept or reject a new goal."""
        self.get_logger().info('Position goal received')
        new_pos: UavPos = goal_request.target_pos

        # Basic validation
        if any(math.isnan(c) for c in new_pos.pos):
             self.get_logger().warn('REJECT: Goal contains NaN values.')
             return GoalResponse.REJECT
        if new_pos.type not in [UavPos.UAV_POS_TYPE_TAKEOFF, UavPos.UAV_POS_TYPE_WAYPOINT, UavPos.UAV_POS_TYPE_LAND, UavPos.UAV_POS_TYPE_PAYLOAD_DROP]:
            self.get_logger().warn(f'REJECT: Unknown position command type: {new_pos.type}')
            return GoalResponse.REJECT
        if (new_pos.type == UavPos.UAV_POS_TYPE_LAND) and not self.uav_airborn:
            self.get_logger().warn('REJECT: Cannot land when already on ground')
            return GoalResponse.REJECT
        if (new_pos.type == UavPos.UAV_POS_TYPE_TAKEOFF) and self.uav_airborn:
            self.get_logger().warn('REJECT: Cannot takeoff when already airborne')
            return GoalResponse.REJECT
        if not self.uav_airborn and new_pos.type == UavPos.UAV_POS_TYPE_WAYPOINT:
            self.get_logger().warn('REJECT: Cannot go to waypoint when not airborne')
            self.get_logger().warn(f'requested pos: {new_pos}')
            return GoalResponse.REJECT

        self.get_logger().info(f'ACCEPT: Goal for UavPos: {new_pos.pos}')
        return GoalResponse.ACCEPT

    def cancel_pos_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests."""
        self.get_logger().info('Received cancel request. Switching to hover/loiter.')
        # TODO: Implement a safe hover/loiter command here
        return CancelResponse.ACCEPT

    def execute_pos_cb(self, goal_handle: ServerGoalHandle) -> GoToPos.Result:
        """Execute the accepted goal."""
        self.action_in_progress = True
        self.get_logger().info("Executing goal...")

        try:
            target_pos: UavPos = goal_handle.request.target_pos
            result: GoToPos.Result = GoToPos.Result()

            # --- Pre-flight checks and setup ---
            if self.mission_state == MissionState.MISSION_STATE_TYPE_OFFBOARD:
                if not self.offboard_enable:
                    self.enter_offboard_mode()

                if not self.armed:
                    self.arm_uav()
            
            # --- Main Execution Loop ---
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal was canceled.'
                    self.get_logger().info(result.message)
                    return result

                # Choose handler based on position type
                success: bool = False
                message: str = ""
                if target_pos.type == UavPos.UAV_POS_TYPE_TAKEOFF:
                    success, message = self._handle_takeoff(goal_handle, target_pos)
                elif target_pos.type == UavPos.UAV_POS_TYPE_WAYPOINT:
                    success, message = self._handle_waypoint(goal_handle, target_pos)
                elif target_pos.type == UavPos.UAV_POS_TYPE_LAND:
                    success, message = self._handle_land(goal_handle, target_pos)
                elif target_pos.type == UavPos.UAV_POS_TYPE_PAYLOAD_DROP:
                    # TODO: Implement payload drop logic
                    self.get_logger().info("Payload drop requested. (Logic not yet implemented)")
                    success, message = True, "Payload drop command received (logic not implemented)"
                else:
                    success, message = False, "Unknown position command type"

                # If handler returns, the action is complete (or failed)
                if success:
                    result.success = success
                    result.message = message
                    goal_handle.succeed()
                    self.get_logger().info(f"Goal succeeded: {message}")
                    return result
                elif not success and message: # If failed and message provided
                    result.success = success
                    result.message = message
                    # No explicit abort, just return failure
                    self.get_logger().error(f"Goal failed: {message}")
                    return result
                
                self.rate.sleep()

        finally:
            self.action_in_progress = False
            self.get_logger().info("Goal execution finished.")

    def _handle_takeoff(self, goal_handle: ServerGoalHandle, target_pos: UavPos) -> Tuple[bool, str]:
        """Private method to handle takeoff using VEHICLE_CMD_NAV_TAKEOFF."""
        self.get_logger().info(f'Takeoff initiated: climbing to altitude {-target_pos.pos[2]}m')
        
        cmd_msg: VehicleCommand = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        cmd_msg.param7 = float(-target_pos.pos[2])  # Target altitude (positive value)
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True
        cmd_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(cmd_msg)

        # Monitor altitude
        while rclpy.ok() and math.fabs(self.uav_pos.pos[2] - target_pos.pos[2]) > self.pos_delta:
            self.get_logger().info(f'Climbing: {-self.uav_pos.pos[2]}')
            if goal_handle.is_cancel_requested:
                return False, "Takeoff canceled"
            feedback: GoToPos.Feedback = GoToPos.Feedback(
                distance_remaining=math.fabs(self.uav_pos.pos[2] - target_pos.pos[2]),
                current_pos=self.uav_pos
            )
            goal_handle.publish_feedback(feedback)
            self.rate.sleep()

        self.uav_airborn = True
        self.last_pos_cmd = target_pos
        return True, "Takeoff complete"

    def _handle_waypoint(self, goal_handle: ServerGoalHandle, target_pos: UavPos) -> Tuple[bool, str]:
        """Private method to handle waypoint navigation based on mission state."""
        self.get_logger().info(f'Navigating to waypoint: {target_pos.pos}')
        
        # --- Conditional Waypoint Logic ---
        if self.mission_state == MissionState.MISSION_STATE_TYPE_OFFBOARD:
            # Use Trajectory Setpoints for precise offboard control
            self.get_logger().info("Using Trajectory Setpoints for waypoint navigation.")
            while rclpy.ok() and self.distance_3d(target_pos.pos, self.uav_pos.pos) > self.pos_delta:
                if goal_handle.is_cancel_requested:
                    return False, "Waypoint navigation canceled"

                setpoint_msg: TrajectorySetpoint = TrajectorySetpoint()
                setpoint_msg.position = [target_pos.pos[0], target_pos.pos[1], target_pos.pos[2]]
                setpoint_msg.yaw = target_pos.yaw
                setpoint_msg.timestamp = self.get_clock().now().nanoseconds // 1000
                self.trajectory_setpoint_publisher.publish(setpoint_msg)

                feedback: GoToPos.Feedback = GoToPos.Feedback(
                    distance_remaining=self.distance_3d(target_pos.pos, self.uav_pos.pos),
                    current_pos=self.uav_pos
                )
                goal_handle.publish_feedback(feedback)
                self.rate.sleep()

        elif self.mission_state == MissionState.MISSION_STATE_TYPE_MODE_CONTROL:
            # Use Vehicle Commands to let PX4 handle waypoint navigation
            self.get_logger().info("Using Vehicle Command for waypoint navigation.")
            cmd_msg: VehicleCommand = VehicleCommand()
            cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_WAYPOINT
            cmd_msg.param1 = 0.0  # Hold time in seconds
            cmd_msg.param2 = 0.0  # Acceptance radius in meters
            cmd_msg.param3 = 0.0  # Pass-through
            cmd_msg.param4 = float(target_pos.yaw)
            cmd_msg.param5 = float(target_pos.pos[0]) # Latitude / X
            cmd_msg.param6 = float(target_pos.pos[1]) # Longitude / Y
            cmd_msg.param7 = float(-target_pos.pos[2]) # Altitude (positive)
            cmd_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.vehicle_command_publisher.publish(cmd_msg)

            # Monitor distance to target
            while rclpy.ok() and self.distance_3d(target_pos.pos, self.uav_pos.pos) > self.pos_delta:
                if goal_handle.is_cancel_requested:
                    return False, "Waypoint navigation canceled"
                feedback: GoToPos.Feedback = GoToPos.Feedback(
                    distance_remaining=self.distance_3d(target_pos.pos, self.uav_pos.pos),
                    current_pos=self.uav_pos
                )
                goal_handle.publish_feedback(feedback)
                self.rate.sleep()
        else:
            return False, f"Unknown mission state for waypoint navigation: {self.mission_state}"

        self.last_pos_cmd = target_pos
        return True, f"Waypoint reached: {target_pos.pos}"

    def _handle_land(self, goal_handle: ServerGoalHandle, target_pos: UavPos) -> Tuple[bool, str]:
        """Private method to handle landing using VEHICLE_CMD_NAV_LAND."""
        self.get_logger().info('Landing initiated')
        
        cmd_msg: VehicleCommand = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(cmd_msg)

        # Monitor vehicle status until disarmed
        while rclpy.ok() and self.armed:
             if goal_handle.is_cancel_requested:
                 return False, "Landing canceled"
             self.rate.sleep()

        self.uav_airborn = False
        self.last_pos_cmd = target_pos
        return True, "Landing complete"

    def maintain_offboard_cb(self) -> None:
        """Maintain offboard control by sending periodic signals."""
        if self.offboard_enable:
            self.send_offboard_signal()
            # If armed but no action is running, hold position (loiter)
            if not self.action_in_progress and self.armed:
                setpoint_msg: TrajectorySetpoint = TrajectorySetpoint()
                # Use last commanded position or current position if none exists
                hold_pos: UavPos = self.last_pos_cmd if len(self.last_pos_cmd.pos) > 0 else self.uav_pos
                setpoint_msg.position = hold_pos.pos
                setpoint_msg.yaw = hold_pos.yaw
                setpoint_msg.timestamp = self.get_clock().now().nanoseconds // 1000
                self.trajectory_setpoint_publisher.publish(setpoint_msg)

    def mission_state_cb(self, msg: MissionState) -> None:
        self.mission_state = msg.state
        self.get_logger().info(f'Mission state updated: {self.mission_state}')

    def status_cb(self, status: VehicleStatus) -> None:
        # arming_state == 2 means ARMED
        self.armed = True if status.arming_state == 2 else False

    def vehicle_control_mode_cb(self, mode: VehicleControlMode) -> None:
        self.offboard_enable = mode.flag_control_offboard_enabled

    def vehicle_local_position_cb(self, local_pos: VehicleLocalPosition) -> None:
        self.uav_pos.pos = [local_pos.x, local_pos.y, local_pos.z]
        self.uav_pos.stamp = local_pos.timestamp

    def distance_3d(self, pos1: list, pos2: list) -> float:
        """Calculate 3D Euclidean distance."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)

    def distance_2d(self, pos1: list, pos2: list) -> float:
        """Calculate 2D Euclidean distance."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def main(args=None) -> None:
    print('Starting UAV Control program...')
    
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node, MultiThreadedExecutor()) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()