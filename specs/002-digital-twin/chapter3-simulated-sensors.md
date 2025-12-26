# Chapter 3: Simulated Sensors - LiDAR, Depth Cameras, IMUs

## Table of Contents
1. [Introduction to Simulated Sensors](#introduction)
2. [LiDAR Simulation](#lidar-simulation)
3. [Depth Camera Simulation](#depth-camera-simulation)
4. [IMU Simulation](#imu-simulation)
5. [Sensor Fusion in Simulation](#sensor-fusion)
6. [ROS 2 Sensor Integration](#ros-integration)
7. [Performance Optimization](#performance-optimization)
8. [Validation and Calibration](#validation-calibration)
9. [Practical Exercises](#practical-exercises)
10. [Troubleshooting and Best Practices](#troubleshooting-best-practices)

## Introduction {#introduction}

Simulated sensors are critical components of digital twin environments, providing synthetic data that mimics real-world sensor behavior. In robotics applications, accurate sensor simulation enables:
- Safe testing of perception algorithms without hardware risk
- Reproducible experiments with controlled environmental conditions
- Training of AI models with synthetic data
- Development of sensor fusion algorithms
- Validation of robot behaviors in complex scenarios

This chapter focuses on three fundamental sensor types commonly used in robotics: LiDAR for 3D mapping and localization, depth cameras for 3D perception, and IMUs for orientation and motion sensing.

### Sensor Simulation Principles

#### Physics-Based Simulation
Realistic sensor simulation requires accurate modeling of:
- Physical sensing principles (ray tracing for LiDAR, projection for cameras, acceleration measurement for IMUs)
- Noise characteristics and sensor limitations
- Environmental factors (lighting, weather, etc.)
- Temporal dynamics and timing constraints

#### Integration with Simulation Pipeline
Simulated sensors must integrate seamlessly with:
- Physics engines for accurate world interactions
- Rendering pipelines for visual sensors
- Robot kinematics for proper mounting and motion
- ROS 2 communication for standard interfaces

## LiDAR Simulation {#lidar-simulation}

### LiDAR Physics and Operation

LiDAR (Light Detection and Ranging) sensors emit laser pulses and measure the time of flight to determine distances to objects. In simulation, this is typically implemented using ray tracing against the 3D scene geometry.

#### Key LiDAR Parameters
- **Range:** Maximum and minimum detection distances
- **Field of View (FOV):** Horizontal and vertical angular coverage
- **Resolution:** Angular resolution between measurements
- **Update Rate:** Frequency of complete scans
- **Number of Beams:** For multi-beam LiDAR systems

### Gazebo LiDAR Implementation

#### GPU Ray Sensor Configuration
```xml
<!-- GPU Ray Sensor (LiDAR) Configuration -->
<sensor name="gpu_lidar" type="gpu_ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>

  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
        <max_angle>3.14159</max_angle>    <!-- 180 degrees -->
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>

  <plugin filename="libgazebo_ros_ray_sensor.so" name="gpu_lidar_ros">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>laser_frame</frame_name>
  </plugin>
</sensor>
```

#### Multi-Beam LiDAR (3D LiDAR)
```xml
<!-- 3D LiDAR Configuration -->
<sensor name="velodyne_vlp16" type="gpu_ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <pose>0 0 0.3 0 0 0</pose>
  <visualize>true</visualize>

  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.4</min>
      <max>100.0</max>
      <resolution>0.001</resolution>
    </range>
  </ray>

  <plugin filename="libgazebo_ros_velodyne_gpu_lidar.so" name="vlp16_ros">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=points</remapping>
    </ros>
    <topicName>velodyne_points</topicName>
    <frameName>velodyne</frameName>
    <min_range>0.4</min_range>
    <max_range>100.0</max_range>
    <gaussian_noise>0.008</gaussian_noise>
  </plugin>
</sensor>
```

### Noise Modeling for LiDAR

#### Adding Realistic Noise
```xml
<!-- LiDAR with noise parameters -->
<sensor name="lidar_with_noise" type="gpu_ray">
  <!-- ... other configuration ... -->

  <ray>
    <!-- ... scan configuration ... -->
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>

  <!-- Noise model -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
  </noise>

  <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_with_noise_plugin">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=noisy_scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>noisy_laser_frame</frame_name>
    <!-- Additional noise parameters -->
    <update_rate>10</update_rate>
    <gaussian_noise>0.01</gaussian_noise>
  </plugin>
</sensor>
```

### Unity LiDAR Visualization

#### Point Cloud Rendering
```csharp
// LiDAR Point Cloud Visualization in Unity
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.Msgs;

public class LidarPointCloudVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    public float pointSize = 0.02f;
    public Color pointColor = Color.red;
    public int maxPoints = 10000;
    public bool useDynamicUpdate = true;

    [Header("ROS Settings")]
    public string scanTopic = "/scan";
    public string pointsTopic = "/velodyne_points";

    [Header("Performance Settings")]
    public float updateInterval = 0.1f;
    public bool useObjectPooling = true;

    private List<GameObject> pointObjects;
    private List<Vector3> pointCache;
    private ROSConnection rosConnection;
    private float lastUpdateTime;
    private Material pointMaterial;

    void Start()
    {
        InitializeVisualization();
        SubscribeToLidarTopics();
    }

    void InitializeVisualization()
    {
        pointObjects = new List<GameObject>();
        pointCache = new List<Vector3>();

        // Create material for points
        pointMaterial = new Material(Shader.Find("Sprites/Default"));
        pointMaterial.color = pointColor;

        lastUpdateTime = Time.time;
    }

    void SubscribeToLidarTopics()
    {
        rosConnection = GetComponent<ROSConnection>() ?? FindObjectOfType<ROSConnection>();

        if (rosConnection != null)
        {
            // Subscribe to LaserScan messages
            rosConnection.Subscribe<LaserScan>(scanTopic, OnLaserScanReceived);

            // Subscribe to PointCloud2 messages (for 3D LiDAR)
            rosConnection.Subscribe<PointCloud2>(pointsTopic, OnPointCloudReceived);
        }
    }

    void OnLaserScanReceived(LaserScan scanMsg)
    {
        if (!useDynamicUpdate || Time.time - lastUpdateTime > updateInterval)
        {
            UpdateLidarVisualization(scanMsg);
            lastUpdateTime = Time.time;
        }
    }

    void OnPointCloudReceived(PointCloud2 pointCloudMsg)
    {
        if (!useDynamicUpdate || Time.time - lastUpdateTime > updateInterval)
        {
            Update3DLidarVisualization(pointCloudMsg);
            lastUpdateTime = Time.time;
        }
    }

    void UpdateLidarVisualization(LaserScan scanMsg)
    {
        ClearPreviousPoints();

        // Calculate points from laser scan
        for (int i = 0; i < scanMsg.ranges.Length; i++)
        {
            float range = (float)scanMsg.ranges[i];

            // Skip invalid ranges
            if (range >= scanMsg.range_min && range <= scanMsg.range_max)
            {
                float angle = (float)scanMsg.angle_min + i * (float)scanMsg.angle_increment;

                Vector3 point = new Vector3(
                    range * Mathf.Cos(angle),
                    0,
                    range * Mathf.Sin(angle)
                );

                CreatePointVisualization(point);
            }
        }
    }

    void Update3DLidarVisualization(PointCloud2 pointCloudMsg)
    {
        ClearPreviousPoints();

        // Parse PointCloud2 message and create visualizations
        // This is a simplified example - actual implementation depends on point format
        int pointStep = (int)pointCloudMsg.point_step;
        byte[] data = pointCloudMsg.data;

        // Calculate number of points
        int numPoints = data.Length / pointStep;

        for (int i = 0; i < numPoints && i < maxPoints; i++)
        {
            // Extract x, y, z coordinates from point cloud data
            // This requires proper parsing of the binary data
            // based on the point cloud format definition
            float x = System.BitConverter.ToSingle(data, i * pointStep + 0);
            float y = System.BitConverter.ToSingle(data, i * pointStep + 4);
            float z = System.BitConverter.ToSingle(data, i * pointStep + 8);

            Vector3 point = new Vector3(x, z, -y); // Convert from ROS to Unity coordinates

            if (IsValidPoint(point))
            {
                CreatePointVisualization(point);
            }
        }
    }

    void CreatePointVisualization(Vector3 position)
    {
        GameObject pointObj;

        if (useObjectPooling)
        {
            // Use object pooling to reduce instantiation overhead
            pointObj = GetPooledObject();
        }
        else
        {
            pointObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        }

        pointObj.transform.SetParent(transform);
        pointObj.transform.position = position;
        pointObj.transform.localScale = Vector3.one * pointSize;

        Renderer renderer = pointObj.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = pointMaterial;
        }

        // Remove collider to avoid physics interactions
        Collider collider = pointObj.GetComponent<Collider>();
        if (collider != null)
        {
            DestroyImmediate(collider);
        }

        pointObjects.Add(pointObj);
    }

    GameObject GetPooledObject()
    {
        // Simple object pooling implementation
        // In a production system, use a proper object pool
        return GameObject.CreatePrimitive(PrimitiveType.Sphere);
    }

    bool IsValidPoint(Vector3 point)
    {
        // Check if point is within reasonable bounds
        return !float.IsNaN(point.x) && !float.IsNaN(point.y) && !float.IsNaN(point.z) &&
               !float.IsInfinity(point.x) && !float.IsInfinity(point.y) && !float.IsInfinity(point.z);
    }

    void ClearPreviousPoints()
    {
        foreach (GameObject pointObj in pointObjects)
        {
            if (pointObj != null)
            {
                DestroyImmediate(pointObj);
            }
        }
        pointObjects.Clear();
    }

    void Update()
    {
        // Additional update logic if needed
    }
}
```

## Depth Camera Simulation {#depth-camera-simulation}

### Depth Camera Principles

Depth cameras provide 3D information by measuring the distance from the camera to objects in the scene. Common types include:
- **Stereo cameras:** Use two cameras to triangulate depth
- **Time-of-flight (ToF):** Measure light travel time
- **Structured light:** Project patterns and analyze distortions

### Gazebo Depth Camera Configuration

#### RGB-D Camera Setup
```xml
<!-- RGB-D Camera Configuration -->
<sensor name="rgbd_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <visualize>true</visualize>

  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
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

  <plugin filename="libgazebo_ros_openni_kinect.so" name="camera_controller">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>camera</cameraName>
    <imageTopicName>/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>camera_rgb_optical_frame</frameName>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    <CxPrime>0.0</CxPrime>
    <Cx>320.0</Cx>
    <Cy>240.0</Cy>
    <focalLength>320.0</focalLength>
    <hackBaseline>0.0</hackBaseline>
  </plugin>
</sensor>
```

#### Adding Noise Models
```xml
<!-- Depth Camera with realistic noise -->
<sensor name="noisy_depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <visualize>true</visualize>

  <camera name="noisy_head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <!-- Depth-specific noise -->
    <depth_camera>
      <output>depths</output>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </depth_camera>
  </camera>

  <!-- Noise model for depth measurements -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm depth noise -->
  </noise>

  <plugin filename="libgazebo_ros_depth_camera.so" name="noisy_camera_controller">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>noisy_camera</cameraName>
    <imageTopicName>/noisy_rgb/image_raw</imageTopicName>
    <depthImageTopicName>/noisy_depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/noisy_depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/noisy_rgb/camera_info</cameraInfoTopicName>
    <frameName>noisy_camera_rgb_optical_frame</frameName>

    <!-- Depth noise parameters -->
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
    <distortion_k1>0.0001</distortion_k1>
    <distortion_k2>0.0001</distortion_k2>
    <distortion_k3>0.0001</distortion_k3>
    <distortion_t1>0.0001</distortion_t1>
    <distortion_t2>0.0001</distortion_t2>
  </plugin>
</sensor>
```

### Unity Depth Camera Integration

#### Depth Image Processing
```csharp
// Depth Camera Integration in Unity
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.Msgs;

public class DepthCameraVisualizer : MonoBehaviour
{
    [Header("Camera Display")]
    public RawImage rgbDisplay;
    public RawImage depthDisplay;
    public AspectRatioFitter rgbAspectFitter;
    public AspectRatioFitter depthAspectFitter;

    [Header("ROS Topics")]
    public string rgbTopic = "/camera/rgb/image_raw";
    public string depthTopic = "/camera/depth/image_raw";
    public string cameraInfoTopic = "/camera/rgb/camera_info";

    [Header("Visualization Settings")]
    public bool showPointCloud = true;
    public float depthScale = 1.0f;
    public Color depthColorMin = Color.black;
    public Color depthColorMax = Color.white;

    [Header("Performance Settings")]
    public int textureWidth = 640;
    public int textureHeight = 480;
    public bool useCompression = true;

    private Texture2D rgbTexture;
    private Texture2D depthTexture;
    private ROSConnection rosConnection;
    private CameraInfo cameraInfo;

    void Start()
    {
        InitializeTextures();
        SubscribeToCameraTopics();
    }

    void InitializeTextures()
    {
        // Create RGB texture
        rgbTexture = new Texture2D(textureWidth, textureHeight, TextureFormat.RGB24, false);
        if (rgbDisplay != null)
        {
            rgbDisplay.texture = rgbTexture;
        }

        // Create depth texture
        depthTexture = new Texture2D(textureWidth, textureHeight, TextureFormat.RFloat, false);
        if (depthDisplay != null)
        {
            depthDisplay.texture = depthTexture;
        }

        // Set up aspect ratios
        if (rgbAspectFitter != null)
        {
            rgbAspectFitter.aspectRatio = (float)textureWidth / textureHeight;
        }
        if (depthAspectFitter != null)
        {
            depthAspectFitter.aspectRatio = (float)textureWidth / textureHeight;
        }
    }

    void SubscribeToCameraTopics()
    {
        rosConnection = GetComponent<ROSConnection>() ?? FindObjectOfType<ROSConnection>();

        if (rosConnection != null)
        {
            // Subscribe to RGB image
            rosConnection.Subscribe<Image>(rgbTopic, OnRgbImageReceived);

            // Subscribe to depth image
            rosConnection.Subscribe<Image>(depthTopic, OnDepthImageReceived);

            // Subscribe to camera info
            rosConnection.Subscribe<CameraInfo>(cameraInfoTopic, OnCameraInfoReceived);
        }
    }

    void OnRgbImageReceived(Image rgbMsg)
    {
        if (rgbMsg.width == textureWidth && rgbMsg.height == textureHeight)
        {
            // Convert ROS image to Unity texture
            UpdateRgbTexture(rgbMsg);
        }
    }

    void OnDepthImageReceived(Image depthMsg)
    {
        if (depthMsg.width == textureWidth && depthMsg.height == textureHeight)
        {
            // Convert ROS depth image to Unity texture
            UpdateDepthTexture(depthMsg);
        }
    }

    void OnCameraInfoReceived(CameraInfo infoMsg)
    {
        cameraInfo = infoMsg;
        Debug.Log($"Camera info updated: {infoMsg.width}x{infoMsg.height}");
    }

    void UpdateRgbTexture(Image imageMsg)
    {
        // Convert ROS Image message to Unity Texture2D
        // This implementation assumes RGB8 encoding
        if (imageMsg.encoding == "rgb8" || imageMsg.encoding == "bgr8")
        {
            Color32[] colors = new Color32[imageMsg.width * imageMsg.height];

            // Convert byte array to Color32 array
            for (int i = 0; i < colors.Length; i++)
            {
                if (imageMsg.encoding == "rgb8")
                {
                    colors[i] = new Color32(
                        imageMsg.data[i * 3],
                        imageMsg.data[i * 3 + 1],
                        imageMsg.data[i * 3 + 2],
                        255
                    );
                }
                else if (imageMsg.encoding == "bgr8") // BGR to RGB conversion
                {
                    colors[i] = new Color32(
                        imageMsg.data[i * 3 + 2],  // B -> R
                        imageMsg.data[i * 3 + 1],  // G -> G
                        imageMsg.data[i * 3],      // R -> B
                        255
                    );
                }
            }

            rgbTexture.SetPixels32(colors);
            rgbTexture.Apply();
        }
    }

    void UpdateDepthTexture(Image depthMsg)
    {
        // Convert ROS depth image to Unity Texture2D
        // This implementation assumes 32-bit float depth
        if (depthMsg.encoding == "32FC1")
        {
            // Convert byte array to float array
            float[] depths = new float[depthMsg.width * depthMsg.height];
            for (int i = 0; i < depths.Length; i++)
            {
                // Extract float from byte array (little-endian)
                depths[i] = System.BitConverter.ToSingle(depthMsg.data, i * 4);
            }

            // Convert to color representation for visualization
            Color32[] colors = new Color32[depths.Length];
            float minDepth = float.MaxValue;
            float maxDepth = float.MinValue;

            // Find min/max for normalization
            foreach (float depth in depths)
            {
                if (!float.IsNaN(depth) && !float.IsInfinity(depth))
                {
                    if (depth < minDepth) minDepth = depth;
                    if (depth > maxDepth) maxDepth = depth;
                }
            }

            // Normalize and convert to colors
            for (int i = 0; i < depths.Length; i++)
            {
                float normalizedDepth = 0.0f;
                if (!float.IsNaN(depths[i]) && !float.IsInfinity(depths[i]))
                {
                    normalizedDepth = Mathf.InverseLerp(minDepth, maxDepth, depths[i]);
                    normalizedDepth = Mathf.Clamp01(normalizedDepth);
                }

                colors[i] = Color32.Lerp(depthColorMin, depthColorMax, normalizedDepth);
            }

            depthTexture.SetPixels32(colors);
            depthTexture.Apply();
        }
    }

    // Method to generate point cloud from depth data
    public Vector3[] GeneratePointCloud(Image depthMsg, CameraInfo camInfo)
    {
        if (depthMsg.encoding != "32FC1") return new Vector3[0];

        float[] depths = new float[depthMsg.width * depthMsg.height];
        for (int i = 0; i < depths.Length; i++)
        {
            depths[i] = System.BitConverter.ToSingle(depthMsg.data, i * 4);
        }

        // Camera intrinsic parameters
        float fx = (float)camInfo.p[0]; // Focal length x
        float fy = (float)camInfo.p[5]; // Focal length y
        float cx = (float)camInfo.p[2]; // Principal point x
        float cy = (float)camInfo.p[6]; // Principal point y

        Vector3[] points = new Vector3[depths.Length];
        int validPoints = 0;

        for (int v = 0; v < depthMsg.height; v++)
        {
            for (int u = 0; u < depthMsg.width; u++)
            {
                int idx = v * depthMsg.width + u;
                float depth = depths[idx];

                if (!float.IsNaN(depth) && !float.IsInfinity(depth) && depth > 0)
                {
                    // Convert pixel coordinates to 3D world coordinates
                    float x = (u - cx) * depth / fx;
                    float y = (v - cy) * depth / fy;
                    float z = depth;

                    // Convert from camera coordinates to world coordinates
                    // Assuming camera frame is in optical format (Z forward, X right, Y down)
                    points[validPoints] = new Vector3(x, -y, z);
                    validPoints++;
                }
            }
        }

        // Return only valid points
        Vector3[] validPointCloud = new Vector3[validPoints];
        System.Array.Copy(points, validPointCloud, validPoints);
        return validPointCloud;
    }

    void Update()
    {
        // Additional update logic if needed
    }
}
```

## IMU Simulation {#imu-simulation}

### IMU Physics and Operation

An Inertial Measurement Unit (IMU) typically combines:
- **Accelerometer:** Measures linear acceleration (includes gravity)
- **Gyroscope:** Measures angular velocity
- **Magnetometer:** Measures magnetic field (provides heading reference)

In simulation, IMU data is derived from the robot's kinematic state with added noise models to simulate real sensor characteristics.

### Gazebo IMU Configuration

#### Basic IMU Setup
```xml
<!-- Basic IMU Configuration -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>false</visualize>

  <imu>
    <!-- Accelerometer noise parameters -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </z>
    </angular_velocity>

    <!-- Gyroscope noise parameters -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0175</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00175</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0175</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00175</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0175</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00175</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <body_name>imu_body</body_name>
    <update_rate>100</update_rate>
    <gaussian_noise>0.0175</gaussian_noise>
    <topic_name>imu/data</topic_name>
    <serviceName>imu/service</serviceName>
  </plugin>
</sensor>
```

#### Advanced IMU with Magnetometer
```xml
<!-- IMU with Magnetometer -->
<sensor name="imu_with_magnetometer" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>false</visualize>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0003</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00003</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0003</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00003</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0003</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00003</bias_stddev>
        </noise>
      </z>
    </angular_velocity>

    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0017</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0017</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0017</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <!-- Magnetometer simulation -->
  <plugin filename="libgazebo_ros_magnetometer.so" name="magnetometer_plugin">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu/mag</remapping>
    </ros>
    <frame_name>mag_link</frame_name>
    <topic_name>imu/mag</topic_name>
    <gaussian_noise>1e-6</gaussian_noise>
  </plugin>
</sensor>
```

### Unity IMU Visualization

#### IMU Data Processing and Visualization
```csharp
// IMU Visualization in Unity
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.Msgs;

public class IMUVisualizer : MonoBehaviour
{
    [Header("Visualization Elements")]
    public GameObject orientationIndicator;
    public Text orientationText;
    public Text linearAccelerationText;
    public Text angularVelocityText;

    [Header("ROS Settings")]
    public string imuTopic = "/imu/data";

    [Header("Visualization Settings")]
    public float orientationScale = 1.0f;
    public Color orientationColor = Color.blue;

    [Header("Performance Settings")]
    public float updateInterval = 0.05f;  // 20 Hz update

    private ROSConnection rosConnection;
    private Imu currentImuData;
    private float lastUpdateTime;

    void Start()
    {
        InitializeVisualization();
        SubscribeToIMUTopic();
    }

    void InitializeVisualization()
    {
        lastUpdateTime = Time.time;

        if (orientationIndicator != null)
        {
            Renderer renderer = orientationIndicator.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material.color = orientationColor;
            }
        }
    }

    void SubscribeToIMUTopic()
    {
        rosConnection = GetComponent<ROSConnection>() ?? FindObjectOfType<ROSConnection>();

        if (rosConnection != null)
        {
            rosConnection.Subscribe<Imu>(imuTopic, OnIMUDataReceived);
        }
    }

    void OnIMUDataReceived(Imu imuMsg)
    {
        currentImuData = imuMsg;

        if (Time.time - lastUpdateTime > updateInterval)
        {
            UpdateVisualization();
            lastUpdateTime = Time.time;
        }
    }

    void UpdateVisualization()
    {
        if (currentImuData != null)
        {
            // Update orientation visualization
            UpdateOrientationVisualization();

            // Update text displays
            UpdateTextDisplays();
        }
    }

    void UpdateOrientationVisualization()
    {
        if (orientationIndicator != null && currentImuData != null)
        {
            // Convert ROS quaternion to Unity quaternion
            // ROS uses (x, y, z, w) format, Unity uses (x, y, z, w) but different coordinate system
            Quaternion rosQuat = new Quaternion(
                (float)currentImuData.orientation.x,
                (float)currentImuData.orientation.y,
                (float)currentImuData.orientation.z,
                (float)currentImuData.orientation.w
            );

            // Apply coordinate system conversion if necessary
            // ROS: X forward, Y left, Z up
            // Unity: X right, Y up, Z forward
            Quaternion unityQuat = ConvertROSToUnityQuaternion(rosQuat);

            orientationIndicator.transform.rotation = unityQuat;
            orientationIndicator.transform.localScale = Vector3.one * orientationScale;
        }
    }

    void UpdateTextDisplays()
    {
        if (currentImuData != null)
        {
            // Update orientation text
            if (orientationText != null)
            {
                Vector3 eulerAngles = ConvertROSToUnityQuaternion(new Quaternion(
                    (float)currentImuData.orientation.x,
                    (float)currentImuData.orientation.y,
                    (float)currentImuData.orientation.z,
                    (float)currentImuData.orientation.w
                )).eulerAngles;

                orientationText.text = $"Orientation: {eulerAngles.x:F2}°, {eulerAngles.y:F2}°, {eulerAngles.z:F2}°";
            }

            // Update linear acceleration text
            if (linearAccelerationText != null)
            {
                Vector3 linearAccel = ConvertROSToUnityVector3(new Vector3(
                    (float)currentImuData.linear_acceleration.x,
                    (float)currentImuData.linear_acceleration.y,
                    (float)currentImuData.linear_acceleration.z
                ));

                linearAccelerationText.text =
                    $"Linear Accel: {linearAccel.x:F3}, {linearAccel.y:F3}, {linearAccel.z:F3} m/s²";
            }

            // Update angular velocity text
            if (angularVelocityText != null)
            {
                Vector3 angularVel = ConvertROSToUnityVector3(new Vector3(
                    (float)currentImuData.angular_velocity.x,
                    (float)currentImuData.angular_velocity.y,
                    (float)currentImuData.angular_velocity.z
                ));

                angularVelocityText.text =
                    $"Angular Vel: {angularVel.x:F3}, {angularVel.y:F3}, {angularVel.z:F3} rad/s";
            }
        }
    }

    Quaternion ConvertROSToUnityQuaternion(Quaternion rosQuat)
    {
        // Convert from ROS coordinate system to Unity coordinate system
        // This conversion assumes standard ROS to Unity transformation:
        // ROS: X forward, Y left, Z up
        // Unity: X right, Y up, Z forward

        return new Quaternion(
            rosQuat.y,    // ROS Y -> Unity X
            -rosQuat.z,   // ROS Z -> Unity Y (negated)
            rosQuat.x,    // ROS X -> Unity Z
            rosQuat.w     // W remains the same
        );
    }

    Vector3 ConvertROSToUnityVector3(Vector3 rosVector)
    {
        // Convert from ROS coordinate system to Unity coordinate system
        return new Vector3(
            rosVector.y,    // ROS Y -> Unity X
            -rosVector.z,   // ROS Z -> Unity Y (negated)
            rosVector.x     // ROS X -> Unity Z
        );
    }

    void Update()
    {
        // Additional update logic if needed
    }

    // Method to calculate orientation from acceleration data (for demonstration)
    public Quaternion CalculateOrientationFromAcceleration(Vector3 linearAcceleration)
    {
        // This is a simplified method to estimate orientation from gravity
        // In practice, you'd use sensor fusion algorithms like Madgwick or Mahony
        Vector3 gravity = linearAcceleration; // This includes gravity effect

        // Normalize to get direction of gravity
        Vector3 gravityDir = gravity.normalized;

        // Create a rotation that aligns the Z axis with gravity
        Vector3 up = Vector3.up;
        Vector3 forward = Vector3.forward;

        // If gravity is pointing down (as expected), align accordingly
        if (Vector3.Dot(gravityDir, Vector3.down) > 0.5f)
        {
            return Quaternion.FromToRotation(Vector3.up, -gravityDir);
        }
        else
        {
            return Quaternion.identity;
        }
    }
}
```

## Sensor Fusion in Simulation {#sensor-fusion}

### Multi-Sensor Integration

Sensor fusion combines data from multiple sensors to improve accuracy and robustness. In simulation, this allows for:

#### Kalman Filter Implementation
```csharp
// Simple Kalman Filter for Sensor Fusion
using UnityEngine;

public class SimpleKalmanFilter
{
    private float state;      // Estimated state
    private float errorCovariance;  // Error covariance
    private float processNoise;     // Process noise
    private float measurementNoise; // Measurement noise

    public SimpleKalmanFilter(float initialState, float initialError,
                             float processNoise, float measurementNoise)
    {
        this.state = initialState;
        this.errorCovariance = initialError;
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
    }

    public float Update(float measurement)
    {
        // Prediction step
        // For this simple example, we assume no process model
        // In practice, you would predict based on motion model
        float predictedState = state; // No change in state prediction
        float predictedError = errorCovariance + processNoise;

        // Update step
        float kalmanGain = predictedError / (predictedError + measurementNoise);
        state = predictedState + kalmanGain * (measurement - predictedState);
        errorCovariance = (1 - kalmanGain) * predictedError;

        return state;
    }

    public void Reset(float newState, float newError)
    {
        state = newState;
        errorCovariance = newError;
    }
}

// Extended example for IMU + LiDAR fusion
public class IMULidarFusion : MonoBehaviour
{
    [Header("Fusion Parameters")]
    public float positionProcessNoise = 0.1f;
    public float positionMeasurementNoise = 0.5f;
    public float velocityProcessNoise = 0.1f;
    public float velocityMeasurementNoise = 0.5f;

    private SimpleKalmanFilter positionFilter;
    private SimpleKalmanFilter velocityFilter;

    void Start()
    {
        InitializeFilters();
    }

    void InitializeFilters()
    {
        positionFilter = new SimpleKalmanFilter(0, 1, positionProcessNoise, positionMeasurementNoise);
        velocityFilter = new SimpleKalmanFilter(0, 1, velocityProcessNoise, velocityMeasurementNoise);
    }

    public float FusePositionData(float imuPosition, float lidarPosition, float deltaTime)
    {
        // In a real implementation, you would:
        // 1. Predict state using IMU data (integration)
        // 2. Update with LiDAR measurements
        // 3. Handle coordinate frame transformations

        // Simplified example: use LiDAR for position, IMU for velocity
        float fusedPosition = positionFilter.Update(lidarPosition);
        // Velocity would come from IMU differentiation or prediction

        return fusedPosition;
    }

    public Vector3 FuseOrientationData(Quaternion imuOrientation, Vector3 visionOrientation)
    {
        // For orientation fusion, you might use:
        // - Complementary filter
        // - Extended Kalman Filter
        // - Madgwick filter

        // Simplified: trust IMU for dynamic motion, vision for absolute reference
        return imuOrientation.eulerAngles; // In practice, implement proper fusion
    }
}
```

### Data Synchronization

#### Time Synchronization for Multi-Sensor Data
```csharp
// Multi-Sensor Time Synchronization
using System.Collections.Generic;
using System.Linq;

public class SensorSynchronizer
{
    public struct SensorData
    {
        public double timestamp;
        public object data;
        public string sensorType;

        public SensorData(double timestamp, object data, string sensorType)
        {
            this.timestamp = timestamp;
            this.data = data;
            this.sensorType = sensorType;
        }
    }

    private List<SensorData> bufferedData;
    private double syncWindow; // Time window for synchronization (seconds)
    private double lastSyncTime;

    public SensorSynchronizer(double syncWindow = 0.01) // 10ms window
    {
        this.syncWindow = syncWindow;
        this.bufferedData = new List<SensorData>();
        this.lastSyncTime = 0;
    }

    public void AddSensorData(double timestamp, object data, string sensorType)
    {
        bufferedData.Add(new SensorData(timestamp, data, sensorType));

        // Keep only recent data within the sync window
        double oldestAllowed = timestamp - syncWindow;
        bufferedData.RemoveAll(d => d.timestamp < oldestAllowed);
    }

    public Dictionary<string, object> GetSynchronizedData(double targetTime)
    {
        Dictionary<string, object> syncData = new Dictionary<string, object>();

        foreach (var sensorType in bufferedData.Select(d => d.sensorType).Distinct())
        {
            var sensorData = bufferedData
                .Where(d => d.sensorType == sensorType)
                .OrderByDescending(d => d.timestamp)
                .FirstOrDefault();

            if (sensorData.data != null &&
                Mathf.Abs((float)(sensorData.timestamp - targetTime)) <= syncWindow)
            {
                syncData[sensorType] = sensorData.data;
            }
        }

        return syncData.Count > 0 ? syncData : null;
    }

    public bool HasCompleteSyncSet()
    {
        // Check if we have data from all required sensors within the window
        var sensorTypes = bufferedData.Select(d => d.sensorType).Distinct().ToList();
        var latestTimestamps = new Dictionary<string, double>();

        foreach (var data in bufferedData)
        {
            if (!latestTimestamps.ContainsKey(data.sensorType) ||
                data.timestamp > latestTimestamps[data.sensorType])
            {
                latestTimestamps[data.sensorType] = data.timestamp;
            }
        }

        // Check if all sensors have recent data
        if (latestTimestamps.Count < sensorTypes.Count)
            return false;

        // Check if timestamps are within sync window
        double maxTime = latestTimestamps.Values.Max();
        double minTime = latestTimestamps.Values.Min();

        return (maxTime - minTime) <= syncWindow;
    }
}
```

## ROS 2 Sensor Integration {#ros-integration}

### Standard Sensor Message Types

#### LaserScan Message Processing
```csharp
// Processing LaserScan messages for ROS 2 integration
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.Msgs;
using System.Collections.Generic;

public class LaserScanProcessor
{
    public static float[] ProcessLaserScan(LaserScan scanMsg)
    {
        // Convert ROS LaserScan to Unity-friendly format
        float[] ranges = new float[scanMsg.ranges.Length];

        for (int i = 0; i < scanMsg.ranges.Length; i++)
        {
            ranges[i] = (float)scanMsg.ranges[i];

            // Handle invalid ranges
            if (ranges[i] < scanMsg.range_min || ranges[i] > scanMsg.range_max)
            {
                ranges[i] = float.NaN; // Mark as invalid
            }
        }

        return ranges;
    }

    public static Vector3[] ConvertToPointCloud(LaserScan scanMsg, Transform sensorTransform)
    {
        List<Vector3> points = new List<Vector3>();
        float angle = (float)scanMsg.angle_min;

        for (int i = 0; i < scanMsg.ranges.Length; i++)
        {
            float range = (float)scanMsg.ranges[i];

            if (range >= scanMsg.range_min && range <= scanMsg.range_max)
            {
                // Calculate point in sensor frame
                Vector3 pointInSensorFrame = new Vector3(
                    range * Mathf.Cos(angle),
                    0,
                    range * Mathf.Sin(angle)
                );

                // Transform to world frame
                Vector3 worldPoint = sensorTransform.TransformPoint(pointInSensorFrame);
                points.Add(worldPoint);
            }

            angle += (float)scanMsg.angle_increment;
        }

        return points.ToArray();
    }

    public static float CalculateMinDistance(LaserScan scanMsg)
    {
        float minDist = float.MaxValue;

        foreach (double range in scanMsg.ranges)
        {
            float floatRange = (float)range;
            if (floatRange >= scanMsg.range_min && floatRange <= scanMsg.range_max && floatRange < minDist)
            {
                minDist = floatRange;
            }
        }

        return float.IsInfinity(minDist) ? -1 : minDist; // Return -1 if no valid readings
    }
}
```

#### PointCloud2 Processing
```csharp
// PointCloud2 message processing
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.Msgs;
using System.Collections.Generic;

public class PointCloud2Processor
{
    public static Vector3[] ProcessPointCloud2(PointCloud2 pointCloudMsg)
    {
        // Parse PointCloud2 format
        int width = (int)pointCloudMsg.width;
        int height = (int)pointCloudMsg.height;
        int pointStep = (int)pointCloudMsg.point_step;
        byte[] data = pointCloudMsg.data;

        List<Vector3> points = new List<Vector3>();
        int numPoints = data.Length / pointStep;

        // Find field indices for x, y, z coordinates
        int xIndex = -1, yIndex = -1, zIndex = -1;

        for (int i = 0; i < pointCloudMsg.fields.Count; i++)
        {
            if (pointCloudMsg.fields[i].name == "x") xIndex = (int)pointCloudMsg.fields[i].offset;
            else if (pointCloudMsg.fields[i].name == "y") yIndex = (int)pointCloudMsg.fields[i].offset;
            else if (pointCloudMsg.fields[i].name == "z") zIndex = (int)pointCloudMsg.fields[i].offset;
        }

        if (xIndex == -1 || yIndex == -1 || zIndex == -1)
        {
            Debug.LogError("Could not find x, y, z fields in PointCloud2 message");
            return new Vector3[0];
        }

        // Extract points
        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * pointStep;

            float x = System.BitConverter.ToSingle(data, offset + xIndex);
            float y = System.BitConverter.ToSingle(data, offset + yIndex);
            float z = System.BitConverter.ToSingle(data, offset + zIndex);

            // Validate point
            if (!float.IsNaN(x) && !float.IsNaN(y) && !float.IsNaN(z) &&
                !float.IsInfinity(x) && !float.IsInfinity(y) && !float.IsInfinity(z))
            {
                points.Add(new Vector3(x, z, -y)); // Convert ROS to Unity coordinates
            }
        }

        return points.ToArray();
    }

    public static float[] ExtractIntensityField(PointCloud2 pointCloudMsg)
    {
        // Find intensity field
        int intensityIndex = -1;
        int intensitySize = 0;

        for (int i = 0; i < pointCloudMsg.fields.Count; i++)
        {
            if (pointCloudMsg.fields[i].name == "intensity")
            {
                intensityIndex = (int)pointCloudMsg.fields[i].offset;
                intensitySize = (int)pointCloudMsg.fields[i].count;
                break;
            }
        }

        if (intensityIndex == -1) return new float[0];

        int pointStep = (int)pointCloudMsg.point_step;
        byte[] data = pointCloudMsg.data;
        int numPoints = data.Length / pointStep;
        float[] intensities = new float[numPoints];

        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * pointStep;
            intensities[i] = System.BitConverter.ToSingle(data, offset + intensityIndex);
        }

        return intensities;
    }
}
```

## Performance Optimization {#performance-optimization}

### Efficient Sensor Simulation

#### Multi-Threading for Sensor Processing
```csharp
// Multi-threaded sensor processing
using System.Threading.Tasks;
using System.Collections.Concurrent;

public class MultiThreadedSensorProcessor
{
    private ConcurrentQueue<System.Action> mainThreadTasks;
    private bool processingEnabled;

    public MultiThreadedSensorProcessor()
    {
        mainThreadTasks = new ConcurrentQueue<System.Action>();
        processingEnabled = true;
    }

    public void ProcessLidarDataAsync(float[] ranges, System.Action<Vector3[]> onProcessed)
    {
        Task.Run(() =>
        {
            // Perform heavy computation on background thread
            Vector3[] points = ConvertRangesToPointCloud(ranges);

            // Queue result back to main thread
            mainThreadTasks.Enqueue(() => onProcessed(points));
        });
    }

    public void ProcessDepthImageAsync(byte[] imageData, int width, int height,
                                     System.Action<Texture2D> onProcessed)
    {
        Task.Run(() =>
        {
            // Process depth image on background thread
            Texture2D processedTexture = ProcessDepthOnBackgroundThread(imageData, width, height);

            // Queue result back to main thread
            mainThreadTasks.Enqueue(() => onProcessed(processedTexture));
        });
    }

    Vector3[] ConvertRangesToPointCloud(float[] ranges)
    {
        // Simulate heavy point cloud conversion
        List<Vector3> points = new List<Vector3>();

        for (int i = 0; i < ranges.Length; i++)
        {
            if (!float.IsNaN(ranges[i]))
            {
                float angle = i * 0.01f; // Simplified angle calculation
                Vector3 point = new Vector3(
                    ranges[i] * Mathf.Cos(angle),
                    0,
                    ranges[i] * Mathf.Sin(angle)
                );
                points.Add(point);
            }
        }

        return points.ToArray();
    }

    Texture2D ProcessDepthOnBackgroundThread(byte[] imageData, int width, int height)
    {
        // Process image data without Unity-specific operations
        // This would typically involve image processing algorithms
        return null; // Placeholder
    }

    public void ExecuteMainThreadTasks()
    {
        while (mainThreadTasks.TryDequeue(out System.Action task))
        {
            task?.Invoke();
        }
    }

    void Update()
    {
        ExecuteMainThreadTasks();
    }
}
```

### Memory Management for Large Sensor Data

#### Streaming Sensor Data
```csharp
// Streaming sensor data to manage memory usage
using System.Collections.Generic;

public class SensorDataStreamer
{
    private Queue<float[]> lidarBuffers;
    private Queue<byte[]> imageBuffers;
    private int maxBuffers = 10; // Limit for memory management

    public SensorDataStreamer()
    {
        lidarBuffers = new Queue<float[]>();
        imageBuffers = new Queue<byte[]>();
    }

    public float[] GetLidarBuffer(int size)
    {
        float[] buffer;

        if (lidarBuffers.Count > 0)
        {
            buffer = lidarBuffers.Dequeue();
            if (buffer.Length != size)
            {
                // Buffer size doesn't match, create new one
                buffer = new float[size];
            }
        }
        else
        {
            buffer = new float[size];
        }

        return buffer;
    }

    public void ReturnLidarBuffer(float[] buffer)
    {
        if (lidarBuffers.Count < maxBuffers)
        {
            System.Array.Clear(buffer, 0, buffer.Length); // Clear for security
            lidarBuffers.Enqueue(buffer);
        }
        // Otherwise, let it be garbage collected
    }

    public byte[] GetImageBuffer(int size)
    {
        byte[] buffer;

        if (imageBuffers.Count > 0)
        {
            buffer = imageBuffers.Dequeue();
            if (buffer.Length != size)
            {
                buffer = new byte[size];
            }
        }
        else
        {
            buffer = new byte[size];
        }

        return buffer;
    }

    public void ReturnImageBuffer(byte[] buffer)
    {
        if (imageBuffers.Count < maxBuffers)
        {
            System.Array.Clear(buffer, 0, buffer.Length);
            imageBuffers.Enqueue(buffer);
        }
    }
}
```

## Validation and Calibration {#validation-calibration}

### Sensor Accuracy Validation

#### Comparing Simulation vs. Real Data
```csharp
// Sensor validation and comparison tools
using UnityEngine;
using System.Collections.Generic;

public class SensorValidator : MonoBehaviour
{
    [Header("Validation Settings")]
    public float positionTolerance = 0.05f; // 5cm tolerance
    public float orientationTolerance = 0.5f; // 0.5 degree tolerance
    public float velocityTolerance = 0.01f; // 1cm/s tolerance

    [Header("Reference Data")]
    public bool useGroundTruth = true;
    public Transform groundTruthTransform;

    private Dictionary<string, List<SensorValidationResult>> validationResults;

    void Start()
    {
        validationResults = new Dictionary<string, List<SensorValidationResult>>();
    }

    public SensorValidationResult ValidateLidarData(float[] simulatedRanges, float[] referenceRanges)
    {
        if (simulatedRanges.Length != referenceRanges.Length)
        {
            Debug.LogError("Range array lengths don't match for validation");
            return new SensorValidationResult(false, 0, "Length mismatch");
        }

        float totalError = 0;
        int validComparisons = 0;

        for (int i = 0; i < simulatedRanges.Length; i++)
        {
            if (!float.IsNaN(simulatedRanges[i]) && !float.IsNaN(referenceRanges[i]))
            {
                float error = Mathf.Abs(simulatedRanges[i] - referenceRanges[i]);
                totalError += error;
                validComparisons++;
            }
        }

        float averageError = validComparisons > 0 ? totalError / validComparisons : float.MaxValue;
        bool isValid = averageError < positionTolerance;

        return new SensorValidationResult(isValid, averageError,
            $"Lidar validation: {averageError:F3}m avg error");
    }

    public SensorValidationResult ValidateIMUData(Quaternion simulatedOrientation,
                                                 Quaternion referenceOrientation)
    {
        float angleError = Quaternion.Angle(simulatedOrientation, referenceOrientation);
        bool isValid = angleError < orientationTolerance;

        return new SensorValidationResult(isValid, angleError,
            $"IMU orientation validation: {angleError:F3}° error");
    }

    public SensorValidationResult ValidateDepthImage(Texture2D simulatedDepth,
                                                   Texture2D referenceDepth)
    {
        if (simulatedDepth.width != referenceDepth.width ||
            simulatedDepth.height != referenceDepth.height)
        {
            return new SensorValidationResult(false, float.MaxValue, "Size mismatch");
        }

        Color32[] simPixels = simulatedDepth.GetPixels32();
        Color32[] refPixels = referenceDepth.GetPixels32();

        float totalError = 0;
        int validPixels = 0;

        for (int i = 0; i < simPixels.Length; i++)
        {
            float simDepth = simPixels[i].r / 255.0f; // Assuming normalized encoding
            float refDepth = refPixels[i].r / 255.0f;

            if (simDepth > 0 && refDepth > 0) // Valid depth values
            {
                float error = Mathf.Abs(simDepth - refDepth);
                totalError += error;
                validPixels++;
            }
        }

        float averageError = validPixels > 0 ? totalError / validPixels : float.MaxValue;
        bool isValid = averageError < positionTolerance;

        return new SensorValidationResult(isValid, averageError,
            $"Depth validation: {averageError:F3} avg error");
    }

    public void LogValidationResult(string sensorType, SensorValidationResult result)
    {
        if (!validationResults.ContainsKey(sensorType))
        {
            validationResults[sensorType] = new List<SensorValidationResult>();
        }

        validationResults[sensorType].Add(result);

        if (!result.isValid)
        {
            Debug.LogWarning($"Validation failed for {sensorType}: {result.message}");
        }
    }

    public ValidationSummary GetValidationSummary()
    {
        var summary = new ValidationSummary();

        foreach (var kvp in validationResults)
        {
            var results = kvp.Value;
            int validCount = results.Count(r => r.isValid);
            float averageError = results.Count > 0 ?
                results.Average(r => r.errorValue) : float.MaxValue;

            summary.AddSensorSummary(kvp.Key, validCount, results.Count, averageError);
        }

        return summary;
    }
}

[System.Serializable]
public class SensorValidationResult
{
    public bool isValid;
    public float errorValue;
    public string message;

    public SensorValidationResult(bool isValid, float errorValue, string message)
    {
        this.isValid = isValid;
        this.errorValue = errorValue;
        this.message = message;
    }
}

[System.Serializable]
public class ValidationSummary
{
    public Dictionary<string, SensorSummary> sensorSummaries;

    public ValidationSummary()
    {
        sensorSummaries = new Dictionary<string, SensorSummary>();
    }

    public void AddSensorSummary(string sensorType, int validCount, int totalCount, float averageError)
    {
        sensorSummaries[sensorType] = new SensorSummary
        {
            sensorType = sensorType,
            validCount = validCount,
            totalCount = totalCount,
            averageError = averageError,
            successRate = totalCount > 0 ? (float)validCount / totalCount : 0
        };
    }
}

[System.Serializable]
public class SensorSummary
{
    public string sensorType;
    public int validCount;
    public int totalCount;
    public float averageError;
    public float successRate;
}
```

## Practical Exercises {#practical-exercises}

### Exercise 1: LiDAR Simulation Setup
1. Configure a 2D LiDAR sensor in Gazebo with realistic parameters
2. Subscribe to the `/scan` topic in Unity and visualize the point cloud
3. Add noise models to simulate real sensor characteristics
4. Validate the simulation against known geometric shapes

### Exercise 2: Depth Camera Integration
1. Set up an RGB-D camera in Gazebo with proper calibration
2. Create Unity visualization for both RGB and depth streams
3. Implement point cloud generation from depth data
4. Test with various lighting conditions and surfaces

### Exercise 3: IMU Data Processing
1. Configure IMU with realistic noise parameters
2. Create Unity visualization for orientation and acceleration
3. Implement basic sensor fusion with other sensors
4. Validate orientation accuracy against ground truth

## Troubleshooting and Best Practices {#troubleshooting-best-practices}

### Common Issues

#### LiDAR Issues
- **Symptoms:** Inconsistent ranges, missing obstacles, incorrect angles
- **Solutions:**
  - Verify sensor mounting position and orientation
  - Check collision geometries of environment objects
  - Validate ray intersection parameters
  - Ensure proper coordinate frame transformations

#### Depth Camera Issues
- **Symptoms:** Incorrect depth values, missing data, poor quality
- **Solutions:**
  - Verify camera intrinsic parameters
  - Check near/far clipping planes
  - Validate image encoding and format
  - Ensure proper lighting conditions

#### IMU Issues
- **Symptoms:** Drifting orientation, noisy readings, incorrect gravity
- **Solutions:**
  - Check sensor mounting and calibration
  - Verify noise parameters match real sensors
  - Validate coordinate frame alignment
  - Implement proper sensor fusion

### Best Practices

#### Performance
- Use efficient data structures for sensor data processing
- Implement spatial indexing for large point clouds
- Optimize update rates based on sensor requirements
- Use object pooling for dynamic visualization elements

#### Accuracy
- Validate simulation against real sensor data
- Implement proper noise models based on datasheets
- Use realistic sensor parameters and limitations
- Test across various environmental conditions

#### Integration
- Follow ROS 2 sensor message standards
- Implement proper coordinate frame transformations
- Ensure timing synchronization between sensors
- Provide clear error handling and fallbacks

This chapter has covered the implementation of simulated sensors in the digital twin environment, including LiDAR, depth cameras, and IMUs, with detailed configuration examples and integration patterns.