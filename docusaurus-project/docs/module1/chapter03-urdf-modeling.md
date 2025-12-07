---
sidebar_position: 3
---

# Chapter 3: URDF for Humanoid Robot Modeling

## What is URDF? Unified Robot Description Format

Unified Robot Description Format (URDF) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertia, visual appearance, and collision properties.

URDF is fundamental to robotics simulation and visualization in ROS ecosystems. It allows for:
- Robot visualization in RViz
- Physics simulation in Gazebo
- Kinematic analysis and inverse kinematics
- Collision detection
- Robot state publishing through robot_state_publisher

## Components of URDF: Links, Joints, Transmissions, Gazebo/physical properties

### Links
Links represent rigid bodies in a robot. Each link has:
- **Visual**: How the link appears visually (shape, color, material)
- **Collision**: How the link behaves in collision detection (shape, properties)
- **Inertial**: Physical properties like mass, center of mass, and inertia matrix

### Joints
Joints define the connection between links and specify how they can move relative to each other:
- **Fixed**: No movement allowed
- **Revolute**: Rotational movement around a single axis
- **Continuous**: Continuous rotational movement
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

### Transmissions
Transmissions describe how actuators (motors) are connected to joints, including gear ratios, mechanical properties, and control interfaces.

### Gazebo Properties
Special tags that define how the robot behaves in Gazebo simulation, including physics properties, sensors, and plugins.

## Building a sophisticated humanoid robot URDF from scratch

Let's create a detailed humanoid robot with proper physical properties, transmissions, and Gazebo integration:

```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/advanced_humanoid</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Material definitions -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>

  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Torso with proper inertial properties -->
  <link name="torso">
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.4"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Head with sensor integration -->
  <link name="head">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

  <!-- Head with camera and IMU sensors -->
  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="head_to_camera" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo sensor plugin for head camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="head_camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.0008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="left_hand">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Arm (mirrored from left) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.0008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="right_hand">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
    </collision>
  </link>

  <link name="left_lower_leg">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.18 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.18 0.1 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Leg (mirrored from left) -->
  <link name="right_upper_leg">
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
    </collision>
  </link>

  <link name="right_lower_leg">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <link name="right_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.18 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.18 0.1 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint definitions with proper limits and safety -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.9" soft_upper_limit="1.9"/>
  </joint>

  <joint name="left_shoulder_roll" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="2.5" effort="100.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-0.4" soft_upper_limit="2.4"/>
  </joint>

  <joint name="left_elbow" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="0.1" effort="50.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.9" soft_upper_limit="0.0"/>
  </joint>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.9" soft_upper_limit="1.9"/>
  </joint>

  <joint name="right_shoulder_roll" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-2.5" upper="0.5" effort="100.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-2.4" soft_upper_limit="0.4"/>
  </joint>

  <joint name="right_elbow" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.1" upper="2.0" effort="50.0" velocity="2.0"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="0.0" soft_upper_limit="1.9"/>
  </joint>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.9" soft_upper_limit="0.4"/>
  </joint>

  <joint name="left_hip_roll" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <joint name="left_knee" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.0" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="0.1" soft_upper_limit="1.9"/>
  </joint>

  <joint name="right_hip_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.9" soft_upper_limit="0.4"/>
  </joint>

  <joint name="right_hip_roll" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <joint name="right_knee" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.0" effort="200.0" velocity="1.5"/>
    <safety_controller k_position="100" k_velocity="10" soft_lower_limit="0.1" soft_upper_limit="1.9"/>
  </joint>

  <!-- Transmissions for ros_control -->
  <transmission name="left_shoulder_pitch_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_shoulder_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_shoulder_pitch_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_shoulder_roll_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_shoulder_roll">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_shoulder_roll_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_elbow">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_elbow_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_shoulder_pitch_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_shoulder_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_shoulder_pitch_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_shoulder_roll_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_shoulder_roll">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_shoulder_roll_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_elbow">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_elbow_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_hip_pitch_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_pitch_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_hip_roll_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_roll">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_roll_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_knee_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_knee">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_knee_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_hip_pitch_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_hip_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_hip_pitch_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_hip_roll_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_hip_roll">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_hip_roll_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_knee_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_knee">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_knee_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo-specific configurations -->
  <gazebo reference="torso">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_hand">
    <material>Gazebo/Wood</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_hand">
    <material>Gazebo/Wood</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_upper_leg">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_lower_leg">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="left_foot">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_lower_leg">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <gazebo reference="right_foot">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

</robot>
```

### Advanced Troubleshooting Tips for URDF Models

**Common Issues and Solutions:**
- **Joint Limits**: Always set realistic joint limits based on the physical robot capabilities
- **Inertial Properties**: Use CAD software or physics calculations to determine accurate inertial properties
- **Collision Meshes**: Use simplified collision meshes for better performance in simulation
- **Gazebo Integration**: Ensure all Gazebo plugins are properly configured for physics simulation
- **Transmissions**: Define proper transmissions for ros_control integration
- **Self-Collisions**: Configure self-collision properties to prevent unrealistic interactions
- **Mass Distribution**: Ensure the robot's center of mass is properly positioned for stability

## Visualizing URDF models in RViz

To visualize your URDF model in RViz:

1. **Launch the robot_state_publisher**:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_humanoid.urdf)
   ```

2. **Launch RViz**:
   ```bash
   rviz2
   ```

3. **Configure RViz**:
   - Add a RobotModel display
   - Set the Robot Description parameter to "robot_description"
   - You should now see your humanoid robot model

## Extending URDF for more complex humanoid features (e.g., hands, sensors)

### Adding Hands
To add more complex hands to your humanoid robot, you can add additional links and joints:

```xml
<!-- Add to left hand -->
<link name="left_hand">
  <visual>
    <geometry>
      <box size="0.08 0.06 0.04"/>
    </geometry>
    <material name="skin">
      <color rgba="1 0.8 0.6 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.08 0.06 0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.3"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<joint name="left_wrist" type="revolute">
  <parent link="left_lower_arm"/>
  <child link="left_hand"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.78" upper="0.78" effort="50" velocity="1"/>
</joint>
```

### Adding Sensors
You can also add sensors to your URDF model:

```xml
<!-- Add a camera to the head -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.04 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.04 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo-specific sensor definition -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Exercises: Add a new limb to a humanoid URDF, configure joint limits

### Exercise 1: Add a New Limb
1. Add a tail or additional limb to the humanoid model
2. Define appropriate links, joints, and visual/collision properties
3. Ensure the new limb has realistic joint limits and physical properties

### Exercise 2: Configure Joint Limits
1. Research realistic joint limits for human-like movements
2. Update the joint limits in your URDF to reflect these realistic ranges
3. Test the model in simulation to ensure it moves naturally

## Summary
This chapter covered the fundamentals of URDF (Unified Robot Description Format) for humanoid robot modeling. You learned about the components of URDF including links, joints, and physical properties. We built a complete humanoid robot model from scratch and explored how to visualize it in RViz. The chapter also covered extending URDF models with complex features like hands and sensors, and provided exercises to practice adding new limbs and configuring joint limits. This knowledge is essential for creating realistic robot models for simulation and control applications.