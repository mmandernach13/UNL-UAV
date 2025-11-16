# Next Steps for UNL-UAV Project (2025-11-12)

This document outlines the recommended next steps for the UNL-UAV project. Each step includes details and links to documentation to guide your research and implementation.

---

## 1. Enhance the `px4_control_node` Action Server

**Priority: High**

**Goal:** Make the `GoToPos` action server "smarter" by handling the `type` field in the `uav_interfaces/msg/UavPos` message. This is the most critical step to enable complex missions.

**Implementation Details:**

In `px4_control_node.py`, you will modify the callback function for your `GoToPos` action server.

1.  **Inspect `target_pos.type`:** Inside the action execution callback, get the `type` from the goal message: `goal_handle.request.target_pos.type`.

2.  **Implement Conditional Logic:** Use an `if/elif/else` block to execute different logic for each type.

    *   **`UAV_POS_TYPE_WAYPOINT`:** This is your existing logic. Continue publishing `TrajectorySetpoint` messages to fly to the desired coordinates.

    *   **`UAV_POS_TYPE_TAKEOFF` / `UAV_POS_TYPE_LAND`:** For these, you need to publish a `VehicleCommand` message to the `/fmu/in/vehicle_command` topic.
        *   **Message:** `px4_msgs.msg.VehicleCommand`
        *   **Key Fields to Set:**
            *   `command`: Use `VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF` or `VehicleCommand.VEHICLE_CMD_NAV_LAND`.
            *   `param1`: For takeoff, this is the minimum pitch. You can often leave it as `NaN`.
            *   `param5`, `param6`: Latitude and Longitude. Can be `NaN` if landing at the current spot.
            *   `param7`: Altitude.
            *   `target_system`: Set to `1`.
            *   `target_component`: Set to `1`.
            *   `source_system`: Set to `1`.
            *   `source_component`: Set to `1`.
            *   `from_external`: Set to `True`.
        *   **Research:** Look at the `VehicleCommand.msg` file in `px4_msgs` and search for "PX4 offboard mode takeoff" or "publishing vehicle commands" for examples.

    *   **`UAV_POS_TYPE_PAYLOAD_DROP`:** This requires calling a ROS2 service.
        *   **Action:** Create a client for the `/payload_control` service.
        *   **Logic:** When you receive a `PAYLOAD_DROP` position, call the service and wait for the response. You should only consider the `GoToPos` action successful if the service call succeeds.
        *   **Research:** [ROS2 Tutorial: Writing a service client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-a-Service-Client-Py.html)

---

## 2. Implement a Basic `payload_control_node`

**Priority: Medium**

**Goal:** Create a placeholder `payload_control` service that the `px4_control_node` can call. This allows you to test the full mission flow without needing the final payload hardware.

**Implementation Details:**

In `payload_control_node.py`:

1.  **Create a Service Server:** Use `self.create_service()` to create a server for the `uav_interfaces/srv/PayloadControl` service. The topic name is `/payload_control`.

2.  **Implement the Callback:** The service callback will receive `request` and `response` objects.
    ```python
    # Inside your payload node class
    def payload_service_callback(self, request, response):
        self.get_logger().info(f'Received payload drop request for payload ID: {request.payload_id}')
        # In the future, you would add hardware control logic here.
        response.success = True
        return response
    ```

3.  **Research:** [ROS2 Tutorial: Writing a service server (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-a-Service-Py.html)

---

## 3. Refine the `mission_planner_node` State Machine

**Priority: Medium**

**Goal:** Make the mission planner more robust by properly handling the results of the `GoToPos` action and managing the mission state.

**Implementation Details:**

1.  **Use `send_goal_async`:** When you call the `GoToPos` action, use `send_goal_async`. This returns a "future" object.

2.  **Add a Response Callback:** Add a callback function to the future that will be executed when the goal is accepted or rejected.

3.  **Add a Result Callback:** The goal handle has another future, `get_result_async`, which completes when the action is done. Add a callback to this to check `result.success`.

    ```python
    # Inside your mission planner node
    def goal_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Waypoint reached successfully!')
            # Add logic here to send the next waypoint
        else:
            self.get_logger().error('Failed to reach waypoint!')
            # Add logic here to abort the mission
    ```

4.  **Research:** [ROS2 Tutorial: Writing an action client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-an-Action-Client-Py.html). Pay close attention to handling futures and callbacks.

---

## 4. Introduction to Testing

**Priority: Start Now**

**Goal:** Set up your `mission` package for testing and write a simple unit test to verify your mission file parsing logic.

**Implementation Details:**

1.  **Update `package.xml`:** In your `mission` package, add these lines to enable testing:
    ```xml
    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>python3-pytest</test_depend>
    ```

2.  **Update `setup.py`:** In the `console_scripts` section of your `setup.py` file in the `mission` package, add a new entry point for your test file.
    ```python
    'test_mission_loading = mission.test.test_mission_loading:main',
    ```

3.  **Create the Test File:** Create `ros2_ws/src/mission/test/test_mission_loading.py`.

    ```python
    import unittest
    # Import the function you want to test from your mission planner
    from mission.mission_planner_node import parse_mission_file 

    class TestMissionLoading(unittest.TestCase):

        def test_parsing(self):
            # Create a dummy mission file content
            dummy_content = "0 10.0 10.0 -5.0 1.57\n3 10.0 10.0 -5.0 1.57"
            
            # You might need to write this to a temporary file
            # and pass the file path to your function.
            
            # Call your parsing function
            positions = your_parsing_function(dummy_content) 
            
            # Assert the results
            self.assertEqual(len(positions), 2)
            self.assertEqual(positions[0].type, 0) # TAKEOFF
            self.assertEqual(positions[1].type, 3) # PAYLOAD_DROP
            self.assertEqual(positions[0].pos[0], 10.0)

    if __name__ == '__main__':
        unittest.main()
    ```

4.  **Run the Test:** Build your package with `colcon build --packages-select mission` and then run with `colcon test --packages-select mission`.

5.  **Research:** [ROS2 Guide: How to test a Python package](https://docs.ros.org/en/humble/How-To-Guides/Testing/How-to-test-a-python-package-with-ament_pytest.html).
