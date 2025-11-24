#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
                        VehicleCommandAck, VehicleLocalPosition, VehicleStatus, VehicleControlMode
from uav_interfaces.msg import UavPos, MissionState
from uav_interfaces.action import GoToPos
from mission.mission.mission_planner_node import MissionPlannerClientNode
import math


class PositionController(Node):

    mission_states_str_to_int = MissionPlannerClientNode.mission_states_str_to_int
    mission_states_int_to_str = MissionPlannerClientNode.mission_states_int_to_str
    pos_types_str_to_int = MissionPlannerClientNode.pos_types_str_to_int
    pos_types_int_to_str = MissionPlannerClientNode.pos_types_int_to_str

    def __init__(self): 
        super().__init__('position_control')
        
        # QoS Profile - Quality of Service settings for reliable PX4 communication
        # BEST_EFFORT: Don't guarantee delivery (faster, PX4 uses this)
        # TRANSIENT_LOCAL: New subscribers get last published message
        # KEEP_LAST with depth=1: Only keep most recent message
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # State variables to track UAV status
        self.uav_pos = UavPos()  # Current [x, y, z], yaw position of UAV
        self.offboard_signal_msg = OffboardControlMode()
        self.mission_state = None  # Current mission state
        self.last_command_ack = None  # Last command acknowledgement from PX4

        # Flags to track UAV condition
        self.uav_airborn = False   # Flag: Has UAV reached takeoff altitude?
        self.offboard_enable = False  # Flag: Is offboard mode active?
        self.auto_enable = False  # Flag: Is auto mode active?
        self.armed = False  # Flag: Are motors armed?
        self.action_in_progress = False  # Flag: Is an action currently being executed?

        # Parameters
        self.declare_parameter('cmd_rate', 10.0)
        self.cmd_freq = self.get_parameter('cmd_rate').value
        self.rate = self.create_rate(self.cmd_freq)

        self.declare_parameter('offboard_rate', 5.0)
        self.offboard_rate = self.get_parameter('offboard_rate').value 

        self.declare_parameter('pos_delta', 0.3)
        self.pos_delta = self.get_parameter('pos_delta').value 

        # ACTION SERVER - Handle incoming position goals
        self.action_server = ActionServer(
            self, 
            GoToPos,
            'go_to_position',
            goal_callback=self.goal_pos_cb,
            cancel_callback=self.cancel_pos_cb,
            execute_callback=self.excecute_pos_cb,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Action server started')

        # TIMERS - Periodic callbacks
        # Maintain offboard control mode by sending signal and hold position at a fixed rate
        self.maintain_offboard_timer = self.create_timer(1/self.offboard_rate, self.maintain_offboard_cb)
        self.get_logger().info('Offboard maintainer started')

        # SUBSCRIBERS - Listen to PX4 status and position
        # Vehicle position in local North-East-Down (NED) coordinate frame
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_local_position_cb, 
            qos_profile
        )
        self.get_logger().info('Position sub created')

        # Vehicle control mode (tells us if offboard mode is active)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_cb, 
            qos_profile
        )
        self.get_logger().info('Control mode sub created')

        # Vehicle status (arming state, flight mode, etc.)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v1', 
            self.status_cb, 
            qos_profile
        )
        self.get_logger().info('Status sub created')

        # Mission state updates
        self.mission_state_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_state_cb,
            qos_profile
        )
        self.get_logger().info('Mission state sub created')

        # Acknowledgements from PX4 after sending commands
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_cb,
            qos_profile
        )
        self.get_logger().info('Command ack sub created')

        # PUBLISHERS - Send commands and setpoints to PX4
        # Send high-level commands (arm, disarm, mode changes, land, etc.)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        self.get_logger().info('Command pub created')

        # Tell PX4 what control mode we want (position, velocity, attitude, etc.)
        # Must publish this at >2Hz or PX4 will exit offboard mode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )
        self.get_logger().info('Control mode pub created')

        # Send position/velocity setpoints for the UAV to follow
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        self.get_logger().info('Trajectory pub created')

    def _set_cmd_system(self, cmd_msg: VehicleCommand) -> VehicleCommand:
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True
        return cmd_msg  

    def send_offboard_signal(self) -> None:
        self.offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(self.offboard_signal_msg)

    def enter_offboard_mode(self) -> None:
        # Create offboard control mode message
        # This tells PX4 we want to control POSITION (not velocity/attitude/etc)
        self.get_logger().info('Going into OFFBOARD mode') 
        self.offboard_signal_msg.position = True  # We'll send position setpoints
        self.offboard_signal_msg.velocity = False
        self.offboard_signal_msg.acceleration = False
        self.offboard_signal_msg.attitude = False
        self.offboard_signal_msg.body_rate = False

        self.send_offboard_signal()

        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd_msg.param1 = 1.0  # Custom mode enabled
        cmd_msg.param2 = 6.0  # Offboard mode (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        cmd_msg = self._set_cmd_system(cmd_msg)

        itr = 0

        while(self.offboard_enable == False):
            self.get_logger().info('Requesting OFFBOARD mode')
            # Wait a bit before requesting (let offboard signal propagate)
            if(itr > 10):
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)
            itr = itr + 1 
            self.rate.sleep()

        self.get_logger().info('OFFBOARD mode enabled')

    def enter_auto_mode(self) -> None:
        self.get_logger().info('Going into AUTO mode') 

        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd_msg.param1 = 1.0  # Custom mode enabled
        cmd_msg.param2 = 4.0  # Auto mode (PX4_CUSTOM_MAIN_MODE_AUTO)
        cmd_msg = self._set_cmd_system(cmd_msg)

        while(self.auto_enable == False):
            self.get_logger().info('Requesting AUTO mode')
            cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(cmd_msg)
            self.rate.sleep()

        self.get_logger().info('AUTO mode enabled')
        
    def arm_uav(self) -> None:
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
        self.get_logger().info('Disarming UAV')
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 0.0 # 1 to arm, 0 to disarm
        
        max_attempts = 100
        attempts = 0
        
        while (self.armed == True) and (attempts < max_attempts):
            self.get_logger().info(f'Attempting to disarm... (attempt {attempts})')

            cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(cmd_msg)
           
            attempts += 1
            self.rate.sleep()
        
        if not self.armed:
            self.get_logger().info('UAV disarmed')
        else:
            self.get_logger().error('Failed to disarm! Check PX4 console')

    # Every new received goal will be processed here first
    # We can decide to accept or reject the incoming goal
    def goal_pos_cb(self, goal_request: GoToPos.Goal) -> GoalResponse:
        self.get_logger().info('Position goal received')

        new_pos: UavPos = goal_request.target_pos
        if (new_pos.pos[0] == self.uav_pos.pos[0]) and \
           (new_pos.pos[1] == self.uav_pos.pos[1]) and \
           (new_pos.pos[2] == self.uav_pos.pos[2]):
            self.get_logger().info('REJECT already at requested position')
            return GoalResponse.REJECT
        elif new_pos.pos[2] > 0:
            self.get_logger().info('REJECT height request is out of bounds')
            return GoalResponse.REJECT
        # elif new_pos.stamp.nanosec > self.get_clock().now().nanoseconds:
        #     self.get_logger().info('REJECT time stamp of request is in the past')
        #     return GoalResponse.REJECT
        
        self.get_logger().info(f'ACCEPT goal for UavPos: {new_pos.pos}')
        return GoalResponse.ACCEPT 

    # Any cancel request will be processed here, we can accept or reject it
    def cancel_pos_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info('Received cancel request')
        # put in hover or land mode
        return CancelResponse.ACCEPT

    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    # When executing the goal we also check if we need to cancel it
    def excecute_pos_cb(self, goal: ServerGoalHandle) -> GoToPos.Result:
        self.action_in_progress = True
        self.take_off = False

        target_pos: UavPos = goal.request.target_pos
        result = GoToPos.Result()

        self.get_logger().info("Wait for uav initial position...")
        while (len(self.uav_pos.pos) == 0):
             time.sleep(0.1)
        self.get_logger().info("position received")

        try:
            if self.mission_state == MissionState.MISSION_STATE_TYPE_OFFBOARD:
                if (self.offboard_enable == False):
                    self.enter_offboard_mode()

            if self.mission_state == MissionState.MISSION_STATE_TYPE_MODE_CONTROL:
                if (self.auto_enable == False):
                    self.enter_auto_mode()

            if (self.armed == False):
                self.arm_uav()

            self.get_logger().info(f'Executing Goal of type: {self.pos_types_int_to_str.get(target_pos.type)}')
            
            while rclpy.ok():
                result = self._check_for_cancellation(goal)

                if result.success == False:
                    return result
                
                if target_pos.type == UavPos.UAV_POS_TYPE_TAKEOFF:
                    result = self._handle_takeoff(goal)
                elif target_pos.type == UavPos.UAV_POS_TYPE_WAYPOINT:   
                    result = self._handle_waypoint(target_pos)
                elif target_pos.type == UavPos.UAV_POS_TYPE_LAND:
                    result = self._handle_landing(goal)

                if result.success:
                    self.get_logger().info('Goal succeeded')
                    goal.succeed()
                    return result
        finally:
            self.action_in_progress = False


    def _check_for_cancellation(self, goal: ServerGoalHandle) -> GoToPos.Result:
        result = GoToPos.Result()

        if goal.is_cancel_requested:
            goal.canceled()
            self.get_logger().info('Goal canceled')
            result.success = False
            result.message = 'Goal canceled'

        return result

    # HANDLE: Takeoff procedure
    # Sends takeoff command and waits until UAV reaches target altitude
    # Publishes feedback on current altitude and distance remaining    
    def _handle_takeoff(self, goal: ServerGoalHandle) -> GoToPos.Result:
        target_pos: UavPos = goal.request.target_pos
        target_altitude = target_pos.pos[2]

        result = GoToPos.Result()
        feedback = GoToPos.Feedback()

        self.get_logger().info(f'Initiating takeoff to altitude {-target_altitude} m')
        
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param7 = -target_altitude  # Takeoff altitude (NED frame, negative down)

        while rclpy.ok() and (self.last_command_ack is None or
              self.last_command_ack.command != VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF or
              self.last_command_ack.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED):
            self.get_logger().info('Sending takeoff command')

            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(msg)
            self.rate.sleep()

        self.get_logger().info('Takeoff command accepted')

        # Wait until UAV reaches target altitude
        while rclpy.ok():
            result = self._check_for_cancellation(goal)

            current_altitude = self.uav_pos.pos[2]
            altitude_error = abs(current_altitude - target_altitude)
            self.get_logger().info(f'Current altitude: {-current_altitude} m, Target altitude: {-target_altitude} m, Error: {altitude_error} m')

            feedback.current_pos = self.uav_pos
            feedback.distance_remaining = altitude_error
            goal.publish_feedback(feedback)

            if altitude_error <= self.pos_delta:
                self.get_logger().info('Takeoff complete')
                self.uav_airborn = True
                result.success = True
                result.message = 'Takeoff successful'
                return result

            self.rate.sleep()
    
    # HANDLE: Navigate to waypoint
    def _handle_waypoint(self, target_pos: UavPos) -> GoToPos.Result:
        self.get_logger().info(f'Navigating to waypoint at position {target_pos.pos} with yaw {target_pos.yaw}')
        # Implement waypoint navigation logic
        pass

    # HANDLE: Landing procedure
    # Sends land command and waits until UAV hits the ground
    # Publishes feedback on current position and distance remaining
    def _handle_landing(self, goal: ServerGoalHandle) -> GoToPos.Result:
        self.get_logger().info('Initiating landing sequence')
        result = GoToPos.Result()
        feedback = GoToPos.Feedback()

        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND

        while rclpy.ok() and (self.last_command_ack is None or
              self.last_command_ack.command != VehicleCommand.VEHICLE_CMD_NAV_LAND or
              self.last_command_ack.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED):
            self.get_logger().info('Sending land command')

            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(msg)
            self.rate.sleep()
        
        self.get_logger().info('Land command accepted')

        # Wait until UAV lands (altitude near zero)
        while rclpy.ok():
            result = self._check_for_cancellation(goal)

            current_altitude = self.uav_pos.pos[2]
            self.get_logger().info(f'Current altitude: {-current_altitude} m')

            feedback.current_pos = self.uav_pos
            feedback.distance_remaining = abs(current_altitude)
            goal.publish_feedback(feedback)

            if abs(current_altitude) <= self.pos_delta:
                self.get_logger().info('Landing complete')
                self.uav_airborn = False
                result.success = True
                result.message = 'Landing successful'
                return result

            self.rate.sleep()

    # CALLBACK: maintains offboard control (have to publish at >2Hz)
    def maintain_offboard_cb(self):
        if self.offboard_enable:
            self.send_offboard_signal()

            if not self.action_in_progress and self.armed:
                # If no action is in progress, hold current position
                msg = TrajectorySetpoint()
                msg.position = self.uav_pos.pos
                msg.yaw = 0.0
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_setpoint_publisher.publish(msg)

    # CALLBACK: Process command acknowledgements from PX4
    def command_ack_cb(self, ack_msg):
        if ack_msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(f'Command {ack_msg.command} acknowledged: ACCEPTED')
        elif ack_msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            self.get_logger().warn(f'Command {ack_msg.command} acknowledged: TEMPORARILY REJECTED')
        elif ack_msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED:
            self.get_logger().error(f'Command {ack_msg.command} acknowledged: DENIED')
        elif ack_msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED:
            self.get_logger().error(f'Command {ack_msg.command} acknowledged: UNSUPPORTED')
        elif ack_msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED:
            self.get_logger().error(f'Command {ack_msg.command} acknowledged: FAILED')

        self.last_command_ack = ack_msg

    # CALLBACK: Update mission state
    def mission_state_cb(self, state_msg):
        self.mission_state = state_msg.state
        self.get_logger().info(f"Mission state updated to: {self.mission_states_int_to_str.get(self.mission_state)}")

    # CALLBACK: Update armed status from vehicle status
    def status_cb(self, status):
        # arming_state == 2 means ARMED
        self.armed = True if status.arming_state == 2 else False
        
    # CALLBACK: Update offboard mode status
    def vehicle_control_mode_cb(self, mode):
        self.offboard_enable = mode.flag_control_offboard_enabled
        self.auto_enable = mode.flag_control_auto_enabled

    # CALLBACK: Update current position
    def vehicle_local_position_cb(self, local_pos):
        # Store position in NED frame (North, East, Down)
        self.uav_pos.pos = [local_pos.x, local_pos.y, local_pos.z]
        self.uav_pos.stamp = local_pos.timestamp

    def distance_2d(self, pos1, pos2):
        x1, y1, _ = pos1[0], pos1[1], pos1[2]
        x2, y2, _ = pos2[0], pos2[1], pos2[2]
        
        dx = x2 - x1
        dy = y2 - y1
        
        return math.sqrt(dx*dx + dy*dy)

    def distance_3d(self, pos1, pos2):
        x1, y1, z1 = pos1[0], pos1[1], pos1[2]
        x2, y2, z2 = pos2[0], pos2[1], pos2[2]
        
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

def main(args=None):
    print('Starting UAV Control program...')
    
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node, MultiThreadedExecutor()) 
    rclpy.shutdown()

if __name__ == '__main__':
    main()