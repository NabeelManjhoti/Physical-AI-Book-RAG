---
sidebar_position: 4
---

# URDF: Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, inertial properties, and visual representations.

## URDF Structure

### Links

Links represent rigid bodies in the robot. Each link has:

- **Visual**: How the link appears in visualization tools
- **Collision**: How the link interacts in collision detection
- **Inertial**: Mass, center of mass, and inertia tensor

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="1 2 3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="1 2 3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define their relative motion:

- **Fixed**: No relative motion
- **Revolute**: Single-axis rotation with limits
- **Continuous**: Single-axis rotation without limits
- **Prismatic**: Single-axis translation with limits
- **Planar**: Motion on a plane
- **Floating**: 6 DOF motion

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Geometry Types

URDF supports several geometric shapes:

- **Box**: Defined by size="x y z"
- **Cylinder**: Defined by radius and length
- **Sphere**: Defined by radius
- **Mesh**: External 3D model files (STL, DAE, etc.)

## Materials

Materials define the visual appearance of links:

```xml
<material name="color_name">
  <color rgba="r g b a"/>
</material>
```

## Complete Robot Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="upper_leg">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_upper_leg" type="fixed">
    <parent link="base_link"/>
    <child link="upper_leg"/>
    <origin xyz="0 0 0.2"/>
  </joint>
</robot>
```

## Xacro: XML Macros

Xacro simplifies complex URDF files by providing macros, properties, and mathematical expressions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <mass value="${m}" />
      <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
               iyy="${m*(3*r*r+h*h)/12}" iyz="0"
               izz="${m*r*r/2}" />
    </inertial>
  </xacro:macro>
</robot>
```

## Best Practices

- Use consistent naming conventions
- Include proper inertial properties for simulation
- Use xacro for complex robots to avoid duplication
- Organize URDF files in the `urdf` directory
- Validate URDF files using `check_urdf` tool
- Use appropriate collision geometry for performance
- Include visual materials for better visualization