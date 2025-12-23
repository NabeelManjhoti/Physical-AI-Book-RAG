---
sidebar_position: 3
---

# Sensor Integration in Simulation

Sensors are critical components of robotic systems, providing the data needed for perception, localization, and mapping. This section covers how to integrate various types of sensors in simulation environments.

## Sensor Types in Robotics

### Range Sensors

Range sensors measure distances to objects in the environment:

- **LIDAR**: Light Detection and Ranging
  - High accuracy range measurements
  - 360-degree coverage in 2D or 3D
  - Essential for mapping and navigation

- **Sonar**: Sound-based distance measurement
  - Lower accuracy than LIDAR
  - Useful for obstacle detection
  - Often used in underwater applications

- **Infrared**: Infrared-based distance sensing
  - Short-range, low-cost option
  - Susceptible to ambient light

### Visual Sensors

Visual sensors capture image data for computer vision applications:

- **Cameras**: RGB image capture
  - Pinhole camera model
  - Distortion parameters
  - Stereo vision for depth

- **Depth Cameras**: RGB-D sensors
  - Color and depth information
  - Point cloud generation
  - Object recognition and pose estimation

### Inertial Sensors

Inertial sensors measure motion and orientation:

- **IMU**: Inertial Measurement Unit
  - Accelerometer: Linear acceleration
  - Gyroscope: Angular velocity
  - Magnetometer: Magnetic field (heading)

- **Encoders**: Position feedback
  - Joint position in manipulators
  - Wheel odometry for mobile robots

## Gazebo Sensor Integration

### Sensor Definition

Sensors in Gazebo are defined within URDF/SDFormat files:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Common Gazebo Sensors

- **Camera**: RGB image capture
- **Depth Camera**: RGB-D sensing
- **LIDAR**: 2D and 3D laser scanning
- **IMU**: Inertial measurement
- **GPS**: Global positioning
- **Force/Torque**: Contact force measurement

## Unity Sensor Integration

### Unity Sensor Architecture

Unity sensors typically use:

- **Camera Components**: For visual sensors
- **Raycasting**: For range sensors
- **Physics Queries**: For contact detection
- **Custom Scripts**: For specialized sensors

### Example: LIDAR Simulation in Unity

```csharp
public class LidarSensor : MonoBehaviour
{
    public int rayCount = 360;
    public float range = 10.0f;

    void Update()
    {
        for (int i = 0; i < rayCount; i++)
        {
            float angle = transform.eulerAngles.y + (i * 360.0f / rayCount);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, range))
            {
                // Process distance measurement
                float distance = hit.distance;
            }
        }
    }
}
```

## Sensor Noise and Uncertainty

### Modeling Real-World Imperfections

Real sensors have various types of noise:

- **Gaussian Noise**: Random measurement errors
- **Bias**: Systematic offset in measurements
- **Drift**: Slowly changing systematic errors
- **Outliers**: Spurious measurements

### Noise Parameters

Sensors should be configured with realistic noise models:

```xml
<camera>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.007</stddev>
  </noise>
</camera>
```

## Sensor Fusion

### Combining Multiple Sensors

Sensor fusion combines data from multiple sensors:

- **Kalman Filters**: Optimal estimation for linear systems
- **Particle Filters**: Non-linear systems with non-Gaussian noise
- **Complementary Filters**: Simple fusion of complementary sensors

### Common Fusion Examples

- **IMU + Odometry**: Improved pose estimation
- **LIDAR + Camera**: Enhanced environment understanding
- **GPS + IMU**: Robust localization

## Best Practices

- Use realistic noise models based on real sensor specifications
- Validate simulation sensors against real hardware when possible
- Consider computational requirements of sensor processing
- Implement proper sensor calibration procedures
- Document sensor parameters and limitations
- Test with various environmental conditions
- Use appropriate update rates for your application