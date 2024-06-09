# issue: dual_kuka_fri


I want to create a dual-iiwa7 arm setup in ROS2 using the FRI stack. I have already created a custom Xacro file that uses the robot namespace to spawn two robots. However, for simplifying the issue, here I will only spawn one robot in my moveit package named "green_moveit_config". Our public Github repo is [here](https://github.com/rysabh/dual_kuka_fri).

In the Xacro file, I have also renamed link and joint names with a prefix to manage both arms in the same environment.

After creating the custom URDF, I used the MoveIt Setup Assistant to generate the MoveIt configurations.

I followed the instructions [here](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_moveit_config/doc/lbr_moveit_config.html#generate-moveit-configuration) in the documentation to modify the controller and other related files.

Additionally, I have remapped certain topics as follows to connect topics from our MoveIt package to topics from the robot driver (bringup.launch.py):

- `{robot_name}_display_planned_path` -> `display_planned_path`
- `{robot_name}_joint_states` -> `joint_states`
- `{robot_name}_monitored_planning_scene` -> `monitored_planning_scene`
- `{robot_name}_planning_scene` -> `planning_scene`
- `{robot_name}_planning_scene_world` -> `planning_scene_world`
- `{robot_name}_robot_description` -> `robot_description`
- `{robot_name}_robot_description_semantic` -> `robot_description_semantic`

However, I am encountering several issues:

**Steps to Reproduce the Error:**

1. Launch the default `bringup.launch.py` with `model:=iiwa7` and `rviz:=false` and `moveit:=false` and `sim:=false`.
2. Launch the custom MoveIt `ros2 launch green_moveit_config demo.launch`.

**Error:**
There is a mismatch in joint names. Planning works, but execution on the real robot fails. 

We observe this when doing a ROS2 service call `/kuka_green/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers` and get the following response:

```
response:
controller_manager_msgs.srv.ListControllers_Response(controller=[
    controller_manager_msgs.msg.ControllerState(name='force_torque_broadcaster', state='active', type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['estimated_ft_sensor/force.x', 'estimated_ft_sensor/force.y', 'estimated_ft_sensor/force.z', 'estimated_ft_sensor/torque.x', 'estimated_ft_sensor/torque.y', 'estimated_ft_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), 
    controller_manager_msgs.msg.ControllerState(name='joint_trajectory_controller', state='active', type='joint_trajectory_controller/JointTrajectoryController', claimed_interfaces=['A1/position', 'A2/position', 'A3/position', 'A4/position', 'A5/position', 'A6/position', 'A7/position'], required_command_interfaces=['A1/position', 'A2/position', 'A3/position', 'A4/position', 'A5/position', 'A6/position', 'A7/position'], required_state_interfaces=['A1/position', 'A1/velocity', 'A2/position', 'A2/velocity', 'A3/position', 'A3/velocity', 'A4/position', 'A4/velocity', 'A5/position', 'A5/velocity', 'A6/position', 'A6/velocity', 'A7/position', 'A7/velocity'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), 
    controller_manager_msgs.msg.ControllerState(name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['A1/commanded_joint_position', 'A1/commanded_torque', 'A1/effort', 'A1/external_torque', 'A1/ipo_joint_position', 'A1/position', 'A1/velocity', 'A2/commanded_joint_position', 'A2/commanded_torque', 'A2/effort', 'A2/external_torque', 'A2/ipo_joint_position', 'A2/position', 'A2/velocity', 'A3/commanded_joint_position', 'A3/commanded_torque', 'A3/effort', 'A3/external_torque', 'A3/ipo_joint_position', 'A3/position', 'A3/velocity', 'A4/commanded_joint_position', 'A4/commanded_torque', 'A4/effort', 'A4/external_torque', 'A4/ipo_joint_position', 'A4/position', 'A4/velocity', 'A5/commanded_joint_position', 'A5/commanded_torque', 'A5/effort', 'A5/external_torque', 'A5/ipo_joint_position', 'A5/position', 'A5/velocity', 'A6/commanded_joint_position', 'A6/commanded_torque', 'A6/effort', 'A6/external_torque', 'A6/ipo_joint_position', 'A6/position', 'A6/velocity', 'A7/commanded_joint_position', 'A7/commanded_torque', 'A7/effort', 'A7/external_torque', 'A7/ipo_joint_position', 'A7/position', 'A7/velocity', 'auxiliary_sensor/client_command_mode', 'auxiliary_sensor/connection_quality', 'auxiliary_sensor/control_mode', 'auxiliary_sensor/drive_state', 'auxiliary_sensor/operation_mode', 'auxiliary_sensor/overlay_type', 'auxiliary_sensor/safety_state', 'auxiliary_sensor/sample_time', 'auxiliary_sensor/session_state', 'auxiliary_sensor/time_stamp_nano_sec', 'auxiliary_sensor/time_stamp_sec', 'auxiliary_sensor/tracking_performance', 'estimated_ft_sensor/force.x', 'estimated_ft_sensor/force.y', 'estimated_ft_sensor/force.z', 'estimated_ft_sensor/torque.x', 'estimated_ft_sensor/torque.y', 'estimated_ft_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), 
    controller_manager_msgs.msg.ControllerState(name='lbr_state_broadcaster', state='active', type='lbr_ros2_control/LBRStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['A1/commanded_joint_position', 'A1/commanded_torque', 'A1/effort', 'A1/external_torque', 'A1/ipo_joint_position', 'A1/position', 'A1/velocity', 'A2/commanded_joint_position', 'A2/commanded_torque', 'A2/effort', 'A2/external_torque', 'A2/ipo_joint_position', 'A2/position', 'A2/velocity', 'A3/commanded_joint_position', 'A3/commanded_torque', 'A3/effort', 'A3/external_torque', 'A3/ipo_joint_position', 'A3/position', 'A3/velocity', 'A4/commanded_joint_position', 'A4/commanded_torque', 'A4/effort', 'A4/external_torque', 'A4/ipo_joint_position', 'A4/position', 'A4/velocity', 'A5/commanded_joint_position', 'A5/commanded_torque', 'A5/effort', 'A5/external_torque', 'A5/ipo_joint_position', 'A5/position', 'A5/velocity', 'A6/commanded_joint_position', 'A6/commanded_torque', 'A6/effort', 'A6/external_torque', 'A6/ipo_joint_position', 'A6/position', 'A6/velocity', 'A7/commanded_joint_position', 'A7/commanded_torque', 'A7/effort', 'A7/external_torque', 'A7/ipo_joint_position', 'A7/position', 'A7/velocity', 'auxiliary_sensor/client_command_mode', 'auxiliary_sensor/connection_quality', 'auxiliary_sensor/control_mode', 'auxiliary_sensor/drive_state', 'auxiliary_sensor/operation_mode', 'auxiliary_sensor/overlay_type', 'auxiliary_sensor/safety_state', 'auxiliary_sensor/sample_time', 'auxiliary_sensor/session_state', 'auxiliary_sensor/time_stamp_nano_sec', 'auxiliary_sensor/time_stamp_sec', 'auxiliary_sensor/tracking_performance', 'estimated_ft_sensor/force.x', 'estimated_ft_sensor/force.y', 'estimated_ft_sensor/force.z', 'estimated_ft_sensor/torque.x', 'estimated_ft_sensor/torque.y', 'estimated_ft_sensor/torque.z'], is chainable=False, is chained=False, reference_interfaces=[], chain_connections=[])
])
```

We observe that joints are not renamed here.

**Expected Behavior:**
The system should correctly recognize and manage the custom link and joint names without any mismatches, and the controller manager should operate as expected based on the modified `lbr_controller.yaml` file.

**Actual Behavior:**
Mismatch in link and joint names leading to errors in the system's operation. The controller manager does not reflect the changes expected from the modified configuration.

Additionally we noticed that, when the link and joint names are not changed, and the launch file is launched with default `robot_name:=lbr`, it works. 
