---
sidebar_position: 4
title: "Chapter 3: Simulated Sensors - LiDAR, Depth Cameras, IMUs with ROS 2 Integration"
---

# Chapter 3: Simulated Sensors - LiDAR, Depth Cameras, IMUs with ROS 2 Integration

## Introduction

Simulated sensors are crucial components in digital twin environments, providing the perception capabilities that allow robots to understand and interact with their surroundings. This chapter explores the implementation and integration of three key sensor types: LiDAR for 3D mapping and navigation, depth cameras for RGB-D perception, and IMUs for motion tracking and orientation. We'll cover both the simulation aspects in Gazebo and Unity, as well as the ROS 2 integration for realistic sensor data processing.

### Sensor Simulation in Robotics

Sensor simulation enables:
- Safe testing of perception algorithms without physical hardware
- Reproducible experiments with consistent environmental conditions
- Cost-effective development and validation of robotic systems
- Accelerated training for machine learning applications
- Integration testing of complete robotic systems

## LiDAR Simulation

### Understanding LiDAR Technology

LiDAR (Light Detection and Ranging) sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This creates a 3D point cloud representation of the environment.

#### Key LiDAR Parameters
- **Range**: Maximum distance the sensor can detect objects
- **Field of View (FOV)**: Angular coverage of the sensor
- **Resolution**: Angular precision of measurements
- **Scan Rate**: How frequently the sensor updates
- **Number of Beams**: Vertical resolution for 3D sensors

### LiDAR Simulation in Gazebo

#### Configuring LiDAR Sensors in SDF
```xml
<!-- Example: 3D LiDAR sensor configuration -->
<sensor name="lidar_3d" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
        <max_angle>3.14159</max_angle>   <!-- 180 degrees -->
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
        <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_3d_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_3d_frame</frame_name>
  </plugin>
</sensor>
```

#### Point Cloud Generation
```xml
<!-- Example: Point cloud output for 3D LiDAR -->
<sensor name="lidar_3d" type="gpu_ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <topic>points</topic>
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>
        <max_angle>0.2618</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_3d_gpu" filename="libgazebo_ros_gpu_laser.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <frame_name>lidar_3d_frame</frame_name>
  </plugin>
</sensor>
```

### LiDAR Simulation in Unity

#### Creating LiDAR Visualization
```csharp
// Example: LiDAR point cloud visualization in Unity
using UnityEngine;
using System.Collections.Generic;

public class LiDARVisualizer : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public float maxRange = 30.0f;
    public float minRange = 0.1f;
    public float fovHorizontal = 360f;
    public float fovVertical = 45f;
    public int horizontalResolution = 1080;
    public int verticalResolution = 64;

    [Header("Visualization")]
    public GameObject pointPrefab;
    public Material pointMaterial;
    public float pointSize = 0.05f;

    private List<GameObject> pointCloud;
    private Transform robotTransform;

    void Start()
    {
        robotTransform = transform.parent; // Assuming LiDAR is child of robot
        pointCloud = new List<GameObject>();
        InitializePointCloud();
    }

    void InitializePointCloud()
    {
        // Create point cloud visualization
        for (int i = 0; i < horizontalResolution * verticalResolution; i++)
        {
            GameObject point = Instantiate(pointPrefab, transform);
            point.GetComponent<Renderer>().material = pointMaterial;
            point.transform.localScale = Vector3.one * pointSize;
            point.SetActive(false);
            pointCloud.Add(point);
        }
    }

    public void UpdatePointCloud(float[] ranges, float[] intensities)
    {
        int pointIndex = 0;
        for (int v = 0; v < verticalResolution; v++)
        {
            for (int h = 0; h < horizontalResolution; h++)
            {
                if (pointIndex < pointCloud.Count)
                {
                    float range = ranges[pointIndex];
                    if (range >= minRange && range <= maxRange)
                    {
                        // Calculate 3D position based on range and angles
                        float hAngle = Mathf.Deg2Rad * (h * fovHorizontal / horizontalResolution - fovHorizontal / 2);
                        float vAngle = Mathf.Deg2Rad * (v * fovVertical / verticalResolution - fovVertical / 2);

                        Vector3 direction = new Vector3(
                            Mathf.Cos(vAngle) * Mathf.Cos(hAngle),
                            Mathf.Cos(vAngle) * Mathf.Sin(hAngle),
                            Mathf.Sin(vAngle)
                        );

                        Vector3 worldPosition = transform.position + direction * range;
                        pointCloud[pointIndex].transform.position = worldPosition;
                        pointCloud[pointIndex].SetActive(true);

                        // Color based on intensity
                        float intensity = intensities[pointIndex];
                        pointCloud[pointIndex].GetComponent<Renderer>().material.color =
                            Color.Lerp(Color.blue, Color.red, intensity);
                    }
                    else
                    {
                        pointCloud[pointIndex].SetActive(false);
                    }
                    pointIndex++;
                }
            }
        }
    }
}
```

### Processing LiDAR Data in ROS 2

#### LiDAR Data Publisher
```python
# Example: LiDAR data processing in ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Publisher for processed LiDAR data
        self.laser_pub = self.create_publisher(LaserScan, '/processed_scan', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)

        # Subscriber for raw LiDAR data
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/raw_scan',
            self.lidar_callback,
            10
        )

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_data)

        # LiDAR parameters
        self.range_min = 0.1
        self.range_max = 30.0
        self.scan_rate = 10.0

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        # Apply noise model
        noisy_ranges = self.add_noise_to_ranges(msg.ranges)

        # Filter out invalid ranges
        filtered_ranges = self.filter_ranges(noisy_ranges)

        # Create processed message
        processed_msg = LaserScan()
        processed_msg.header = msg.header
        processed_msg.angle_min = msg.angle_min
        processed_msg.angle_max = msg.angle_max
        processed_msg.angle_increment = msg.angle_increment
        processed_msg.time_increment = msg.time_increment
        processed_msg.scan_time = msg.scan_time
        processed_msg.range_min = self.range_min
        processed_msg.range_max = self.range_max
        processed_msg.ranges = filtered_ranges
        processed_msg.intensities = msg.intensities

        # Publish processed data
        self.laser_pub.publish(processed_msg)

    def add_noise_to_ranges(self, ranges):
        """Add realistic noise to LiDAR ranges"""
        noise_std = 0.02  # 2cm standard deviation
        noisy_ranges = []

        for r in ranges:
            if r > self.range_min and r < self.range_max:
                # Add Gaussian noise
                noisy_r = r + np.random.normal(0, noise_std)
                # Add range-dependent noise
                noisy_r += np.random.normal(0, 0.001 * r)
                noisy_ranges.append(max(self.range_min, min(self.range_max, noisy_r)))
            else:
                noisy_ranges.append(r)

        return noisy_ranges

    def filter_ranges(self, ranges):
        """Filter out invalid range values"""
        filtered = []
        for r in ranges:
            if r >= self.range_min and r <= self.range_max:
                filtered.append(r)
            else:
                filtered.append(float('inf'))  # Use inf for invalid readings
        return filtered

    def process_data(self):
        """Additional processing tasks"""
        pass

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LiDARProcessor()

    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Camera Simulation

### Understanding Depth Cameras

Depth cameras provide both color (RGB) and depth (D) information, creating RGB-D data that's essential for 3D scene understanding, object recognition, and manipulation tasks.

#### Key Depth Camera Parameters
- **Resolution**: Image dimensions (width × height)
- **Field of View**: Angular coverage
- **Depth Range**: Minimum and maximum measurable distances
- **Accuracy**: Precision of depth measurements
- **Frame Rate**: How frequently images are captured

### Depth Camera Simulation in Gazebo

#### Configuring Depth Cameras in SDF
```xml
<!-- Example: Depth camera configuration -->
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/image_raw:=image_raw</remapping>
      <remapping>~/camera_info:=camera_info</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_frame</frame_name>
    <hack_baseline>0.07</hack_baseline>
  </plugin>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
      <remapping>~/depth/camera_info:=depth/camera_info</remapping>
      <remapping>~/points:=depth/points</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_frame</frame_name>
    <baseline>0.2</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### Depth Camera Simulation in Unity

#### Creating Depth Camera Visualization
```csharp
// Example: Depth camera visualization in Unity
using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;

public class DepthCameraVisualizer : MonoBehaviour
{
    [Header("Depth Camera Configuration")]
    public int width = 640;
    public int height = 480;
    public float nearClip = 0.1f;
    public float farClip = 10.0f;
    public float fov = 60f;

    [Header("Visualization")]
    public Material depthMaterial;
    public Shader depthShader;

    private Camera depthCamera;
    private RenderTexture depthTexture;
    private Texture2D depthTextureCPU;
    private float[] depthValues;

    void Start()
    {
        SetupDepthCamera();
        CreateDepthVisualization();
    }

    void SetupDepthCamera()
    {
        // Create depth camera
        depthCamera = GetComponent<Camera>();
        if (depthCamera == null)
        {
            depthCamera = gameObject.AddComponent<Camera>();
        }

        depthCamera.fieldOfView = fov;
        depthCamera.nearClipPlane = nearClip;
        depthCamera.farClipPlane = farClip;
        depthCamera.depth = -1; // Render after main camera
        depthCamera.enabled = false; // We'll render manually
    }

    void CreateDepthVisualization()
    {
        // Create render texture for depth
        depthTexture = new RenderTexture(width, height, 24, RenderTextureFormat.Depth);
        depthTexture.Create();

        depthTextureCPU = new Texture2D(width, height, TextureFormat.RFloat, false);
        depthValues = new float[width * height];

        depthCamera.targetTexture = depthTexture;
    }

    public float[] GetDepthData()
    {
        // Render the depth camera
        depthCamera.Render();

        // Read depth texture to CPU
        RenderTexture.active = depthTexture;
        depthTextureCPU.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthTextureCPU.Apply();

        // Convert color values to depth
        Color[] colors = depthTextureCPU.GetPixels();
        for (int i = 0; i < colors.Length; i++)
        {
            // Extract depth value from color (this is a simplified example)
            depthValues[i] = colors[i].r;
        }

        RenderTexture.active = null;
        return depthValues;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if (depthMaterial != null)
        {
            Graphics.Blit(source, destination, depthMaterial);
        }
        else
        {
            Graphics.Blit(source, destination);
        }
    }
}
```

### Processing Depth Camera Data in ROS 2

#### Depth Image Processor
```python
# Example: Depth camera data processing in ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Camera info subscriber
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.image_width = 640
        self.image_height = 480

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)
        self.image_width = msg.width
        self.image_height = msg.height

    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Apply any RGB processing here
            processed_image = self.process_rgb_image(cv_image)

            # Convert back to ROS message and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            self.rgb_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth image and generate point cloud"""
        try:
            # Convert ROS image to OpenCV
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Apply noise and filtering
            noisy_depth = self.add_noise_to_depth(cv_depth)
            filtered_depth = self.filter_depth_image(noisy_depth)

            # Publish processed depth
            processed_depth_msg = self.bridge.cv2_to_imgmsg(filtered_depth, "32FC1")
            processed_depth_msg.header = msg.header
            self.depth_pub.publish(processed_depth_msg)

            # Generate point cloud if camera parameters are available
            if self.camera_matrix is not None:
                pointcloud = self.depth_to_pointcloud(filtered_depth, self.camera_matrix)
                self.pointcloud_pub.publish(pointcloud)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def add_noise_to_depth(self, depth_image):
        """Add realistic noise to depth image"""
        noise_std = 0.01  # 1cm standard deviation
        noise = np.random.normal(0, noise_std, depth_image.shape).astype(np.float32)

        # Add noise only to valid depth values
        valid_mask = (depth_image > 0) & (depth_image < float('inf'))
        noisy_depth = depth_image.copy()
        noisy_depth[valid_mask] += noise[valid_mask]

        # Ensure depth values remain positive
        noisy_depth = np.maximum(noisy_depth, 0.0)

        return noisy_depth

    def filter_depth_image(self, depth_image):
        """Apply filtering to depth image"""
        # Apply median filter to reduce noise
        filtered = cv2.medianBlur(depth_image, 5)

        # Apply bilateral filter for edge-preserving smoothing
        filtered = cv2.bilateralFilter(filtered, 9, 75, 75)

        return filtered

    def depth_to_pointcloud(self, depth_image, camera_matrix):
        """Convert depth image to point cloud"""
        height, width = depth_image.shape

        # Generate coordinate grids
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Convert pixel coordinates to camera coordinates
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        x_coords = (u_coords - cx) * depth_image / fx
        y_coords = (v_coords - cy) * depth_image / fy
        z_coords = depth_image

        # Flatten arrays and combine
        points = np.stack([x_coords.flatten(), y_coords.flatten(), z_coords.flatten()], axis=1)

        # Remove invalid points (where depth is 0 or infinity)
        valid_points = points[np.isfinite(points).all(axis=1)]

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_depth_frame"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        return point_cloud2.create_cloud(header, fields, valid_points)

    def process_rgb_image(self, image):
        """Apply processing to RGB image"""
        # Example: Apply basic image enhancement
        enhanced = cv2.detailEnhance(image)
        return enhanced

def main(args=None):
    rclpy.init(args=args)
    depth_processor = DepthCameraProcessor()

    try:
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        pass
    finally:
        depth_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Simulation

### Understanding IMU Sensors

An Inertial Measurement Unit (IMU) combines accelerometers, gyroscopes, and sometimes magnetometers to measure linear acceleration, angular velocity, and orientation. IMUs are essential for robot localization, balance control, and motion tracking.

#### Key IMU Parameters
- **Accelerometer range**: Maximum measurable acceleration (typically ±2g to ±16g)
- **Gyroscope range**: Maximum measurable angular velocity (typically ±250°/s to ±2000°/s)
- **Magnetometer range**: Magnetic field measurement range
- **Update rate**: How frequently measurements are provided
- **Noise characteristics**: Sensor noise and drift properties

### IMU Simulation in Gazebo

#### Configuring IMU Sensors in SDF
```xml
<!-- Example: IMU sensor configuration -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
    <body_name>imu_link</body_name>
    <update_rate>100</update_rate>
    <gaussian_noise>0.01</gaussian_noise>
  </plugin>
  <pose>0 0 0 0 0 0</pose>
</sensor>

<!-- More detailed IMU configuration with noise parameters -->
<sensor name="detailed_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev> <!-- ~0.1 deg/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-03</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-03</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-03</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="detailed_imu_controller" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <body_name>imu_body</body_name>
  </plugin>
</sensor>
```

### IMU Simulation in Unity

#### Creating IMU Visualization
```csharp
// Example: IMU simulation and visualization in Unity
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class IMUSimulator : MonoBehaviour
{
    [Header("IMU Configuration")]
    public float updateRate = 100.0f; // Hz
    public float accelerometerNoise = 0.001f;
    public float gyroscopeNoise = 0.0017f;
    public float magnetometerNoise = 0.01f;

    [Header("ROS Settings")]
    public string imuTopic = "/imu/data";

    private ROSConnection ros;
    private float lastUpdateTime = 0f;
    private Rigidbody robotRigidbody;

    // Previous state for integration
    private Vector3 prevAngularVelocity = Vector3.zero;
    private Vector3 prevLinearAcceleration = Vector3.zero;
    private Quaternion integratedOrientation = Quaternion.identity;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(imuTopic);

        robotRigidbody = GetComponentInParent<Rigidbody>();
        if (robotRigidbody == null)
        {
            robotRigidbody = GetComponent<Rigidbody>();
        }
    }

    void Update()
    {
        if (Time.time - lastUpdateTime > 1.0f / updateRate)
        {
            PublishIMUData();
            lastUpdateTime = Time.time;
        }
    }

    void PublishIMUData()
    {
        ImuMsg imuMsg = new ImuMsg();

        // Set header
        imuMsg.header = new std_msgs.HeaderMsg();
        imuMsg.header.stamp = new builtin_interfaces.TimeMsg();
        // Note: Time is set automatically by ROS
        imuMsg.header.frame_id = "imu_link";

        // Get current motion data from Unity
        Vector3 angularVelocity = Vector3.zero;
        Vector3 linearAcceleration = Vector3.zero;

        if (robotRigidbody != null)
        {
            // Angular velocity from rigidbody
            angularVelocity = robotRigidbody.angularVelocity;

            // Linear acceleration (calculate from change in velocity)
            linearAcceleration = robotRigidbody.velocity - prevLinearAcceleration;
            prevLinearAcceleration = robotRigidbody.velocity;
        }
        else
        {
            // If no rigidbody, use transform changes
            angularVelocity = CalculateAngularVelocity();
            linearAcceleration = CalculateLinearAcceleration();
        }

        // Add noise to measurements
        angularVelocity += AddGaussianNoise(Vector3.zero, gyroscopeNoise);
        linearAcceleration += AddGaussianNoise(Vector3.zero, accelerometerNoise);

        // Set angular velocity with covariance
        imuMsg.angular_velocity = new Vector3Msg(
            angularVelocity.x,
            angularVelocity.y,
            angularVelocity.z
        );
        imuMsg.angular_velocity_covariance = new double[] {
            gyroscopeNoise * gyroscopeNoise, 0, 0,
            0, gyroscopeNoise * gyroscopeNoise, 0,
            0, 0, gyroscopeNoise * gyroscopeNoise
        };

        // Set linear acceleration with covariance
        imuMsg.linear_acceleration = new Vector3Msg(
            linearAcceleration.x,
            linearAcceleration.y,
            linearAcceleration.z
        );
        imuMsg.linear_acceleration_covariance = new double[] {
            accelerometerNoise * accelerometerNoise, 0, 0,
            0, accelerometerNoise * accelerometerNoise, 0,
            0, 0, accelerometerNoise * accelerometerNoise
        };

        // Calculate orientation (simplified - in real application, integrate properly)
        integratedOrientation = IntegrateOrientation(
            angularVelocity * Mathf.Deg2Rad,
            Time.deltaTime
        );

        imuMsg.orientation = new QuaternionMsg(
            integratedOrientation.x,
            integratedOrientation.y,
            integratedOrientation.z,
            integratedOrientation.w
        );
        imuMsg.orientation_covariance = new double[] {
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01
        };

        // Publish IMU message
        ros.Publish(imuTopic, imuMsg);
    }

    Vector3 AddGaussianNoise(Vector3 mean, float stdDev)
    {
        return new Vector3(
            RandomGaussian(mean.x, stdDev),
            RandomGaussian(mean.y, stdDev),
            RandomGaussian(mean.z, stdDev)
        );
    }

    float RandomGaussian(float mean, float stdDev)
    {
        // Box-Muller transform for Gaussian random numbers
        float u1 = Random.value;
        float u2 = Random.value;
        float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return mean + stdDev * normal;
    }

    Vector3 CalculateAngularVelocity()
    {
        // Calculate angular velocity based on transform changes
        // This is a simplified approach - real implementation would be more complex
        return Vector3.zero;
    }

    Vector3 CalculateLinearAcceleration()
    {
        // Calculate linear acceleration based on transform changes
        // This is a simplified approach - real implementation would track velocity
        return Vector3.zero;
    }

    Quaternion IntegrateOrientation(Vector3 angularVelocity, float deltaTime)
    {
        // Integrate angular velocity to get orientation
        Vector3 rotationVector = angularVelocity * deltaTime;
        float rotationMagnitude = rotationVector.magnitude;

        if (rotationMagnitude > 0)
        {
            Vector3 rotationAxis = rotationVector / rotationMagnitude;
            Quaternion deltaRotation = Quaternion.AngleAxis(
                rotationMagnitude * Mathf.Rad2Deg,
                rotationAxis
            );
            return integratedOrientation * deltaRotation;
        }

        return integratedOrientation;
    }
}
```

### Processing IMU Data in ROS 2

#### IMU Data Processor
```python
# Example: IMU data processing in ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Publisher for processed IMU data
        self.imu_pub = self.create_publisher(Imu, '/processed_imu', 10)

        # Subscriber for raw IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/raw_imu',
            self.imu_callback,
            10
        )

        # Timer for processing
        self.timer = self.create_timer(0.01, self.process_data)  # 100Hz

        # IMU state variables
        self.orientation = R.from_quat([0, 0, 0, 1])  # Initial orientation
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])

        # IMU parameters
        self.acceleration_variance = 0.001**2
        self.angular_velocity_variance = 0.0017**2
        self.orientation_variance = 0.01**2

        # Time tracking
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Extract data from message
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Get current time
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        # Convert to numpy arrays
        linear_acc = np.array([ax, ay, az])
        angular_vel = np.array([gx, gy, gz])

        # Add realistic noise to measurements
        noisy_linear_acc = self.add_noise_to_acceleration(linear_acc)
        noisy_angular_vel = self.add_noise_to_angular_velocity(angular_vel)

        # Update internal state with integration
        self.integrate_imu_data(noisy_angular_vel, noisy_linear_acc, dt)

        # Create processed IMU message
        processed_msg = Imu()
        processed_msg.header = Header()
        processed_msg.header.stamp = current_time.to_msg()
        processed_msg.header.frame_id = msg.header.frame_id

        # Set orientation (from integrated state)
        quat = self.orientation.as_quat()
        processed_msg.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        processed_msg.orientation_covariance = [
            self.orientation_variance, 0.0, 0.0,
            0.0, self.orientation_variance, 0.0,
            0.0, 0.0, self.orientation_variance
        ]

        # Set angular velocity
        processed_msg.angular_velocity = Vector3(
            x=noisy_angular_vel[0],
            y=noisy_angular_vel[1],
            z=noisy_angular_vel[2]
        )
        processed_msg.angular_velocity_covariance = [
            self.angular_velocity_variance, 0.0, 0.0,
            0.0, self.angular_velocity_variance, 0.0,
            0.0, 0.0, self.angular_velocity_variance
        ]

        # Set linear acceleration
        processed_msg.linear_acceleration = Vector3(
            x=noisy_linear_acc[0],
            y=noisy_linear_acc[1],
            z=noisy_linear_acc[2]
        )
        processed_msg.linear_acceleration_covariance = [
            self.acceleration_variance, 0.0, 0.0,
            0.0, self.acceleration_variance, 0.0,
            0.0, 0.0, self.acceleration_variance
        ]

        # Publish processed data
        self.imu_pub.publish(processed_msg)

    def add_noise_to_acceleration(self, acceleration):
        """Add realistic noise to acceleration measurements"""
        noise_std = math.sqrt(self.acceleration_variance)
        noise = np.random.normal(0, noise_std, 3)
        return acceleration + noise

    def add_noise_to_angular_velocity(self, angular_velocity):
        """Add realistic noise to angular velocity measurements"""
        noise_std = math.sqrt(self.angular_velocity_variance)
        noise = np.random.normal(0, noise_std, 3)
        return angular_velocity + noise

    def integrate_imu_data(self, angular_velocity, linear_acceleration, dt):
        """Integrate IMU data to update orientation and velocity"""
        if dt > 0:
            # Integrate angular velocity to update orientation
            # Convert angular velocity to rotation vector
            rotation_vector = angular_velocity * dt
            rotation_magnitude = np.linalg.norm(rotation_vector)

            if rotation_magnitude > 1e-6:  # Avoid division by zero
                rotation_axis = rotation_vector / rotation_magnitude
                # Create small rotation
                small_rotation = R.from_rotvec(rotation_axis * rotation_magnitude)
                # Apply to current orientation
                self.orientation = small_rotation * self.orientation

    def process_data(self):
        """Additional processing tasks"""
        pass

def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()

    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Calibration and Noise Modeling

### Understanding Sensor Calibration

Sensor calibration is crucial for accurate perception in robotics. It involves determining the parameters that relate sensor measurements to real-world values.

#### LiDAR Calibration
- **Intrinsic parameters**: Range accuracy, angular resolution, beam divergence
- **Extrinsic parameters**: Position and orientation relative to robot base
- **Systematic errors**: Range bias, angular offset, beam alignment

#### Camera Calibration
- **Intrinsic parameters**: Focal length, principal point, distortion coefficients
- **Extrinsic parameters**: Position and orientation relative to robot base
- **Distortion models**: Radial and tangential distortion

#### IMU Calibration
- **Bias calibration**: Determine static offsets in measurements
- **Scale factor calibration**: Correct for sensitivity differences
- **Alignment calibration**: Account for misalignment between axes

### Noise Modeling

Real sensors have various types of noise that affect measurement accuracy:

#### Types of Sensor Noise
- **Gaussian noise**: Random variations following a normal distribution
- **Bias**: Systematic offset in measurements
- **Drift**: Slow variation in bias over time
- **Quantization noise**: Discretization errors in digital sensors

#### Implementing Noise Models
```python
# Example: Comprehensive sensor noise modeling
import numpy as np
from scipy import stats

class SensorNoiseModel:
    def __init__(self):
        # LiDAR noise parameters
        self.lidar_range_bias = 0.02  # 2cm bias
        self.lidar_range_noise_std = 0.01  # 1cm standard deviation
        self.lidar_range_noise_scale = 0.001  # Scale with range

        # Camera noise parameters
        self.camera_pixel_noise_std = 0.5  # 0.5 pixel standard deviation
        self.camera_depth_noise_std = 0.01  # 1cm for depth

        # IMU noise parameters
        self.imu_accel_bias = np.array([0.01, -0.02, 0.005])  # m/s²
        self.imu_gyro_bias = np.array([0.001, -0.002, 0.0005])  # rad/s
        self.imu_accel_noise_std = 0.001  # m/s²
        self.imu_gyro_noise_std = 0.0017  # rad/s

    def add_lidar_noise(self, ranges):
        """Add realistic noise to LiDAR ranges"""
        noisy_ranges = []
        for r in ranges:
            if r > 0 and r < float('inf'):
                # Add range-dependent noise
                range_dependent_noise = self.lidar_range_noise_scale * r
                total_noise_std = np.sqrt(
                    self.lidar_range_noise_std**2 + range_dependent_noise**2
                )

                # Add noise: bias + Gaussian noise
                noisy_r = r + self.lidar_range_bias + \
                         np.random.normal(0, total_noise_std)
                noisy_ranges.append(max(0.0, noisy_r))  # Ensure positive
            else:
                noisy_ranges.append(r)
        return noisy_ranges

    def add_camera_noise(self, image, depth_image):
        """Add noise to camera images"""
        # Add noise to RGB image
        rgb_noise = np.random.normal(0, self.camera_pixel_noise_std, image.shape)
        noisy_image = np.clip(image + rgb_noise, 0, 255).astype(np.uint8)

        # Add noise to depth image
        depth_noise = np.random.normal(0, self.camera_depth_noise_std, depth_image.shape)
        noisy_depth = depth_image + depth_noise
        noisy_depth = np.maximum(noisy_depth, 0.0)  # Ensure positive

        return noisy_image, noisy_depth

    def add_imu_noise(self, linear_accel, angular_vel):
        """Add noise to IMU measurements"""
        # Add bias and noise to linear acceleration
        noisy_accel = linear_accel + self.imu_accel_bias + \
                     np.random.normal(0, self.imu_accel_noise_std, 3)

        # Add bias and noise to angular velocity
        noisy_gyro = angular_vel + self.imu_gyro_bias + \
                    np.random.normal(0, self.imu_gyro_noise_std, 3)

        return noisy_accel, noisy_gyro

# Example usage
noise_model = SensorNoiseModel()
</pre>