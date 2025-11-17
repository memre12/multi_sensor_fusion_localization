# Launch Files for Autoware Based Localization System

## Overview

This ROS2 package provides an advanced sensor integration and localization system designed for autonomous robotics and navigation applications.

## Project Structure

```
.
├── CMakeLists.txt              # CMake build configuration
├── config/                     # Configuration directory
│   ├── imu/                    # IMU-specific configurations
│   │   ├── gnss_poser.param.yaml           # GNSS positioning parameters
│   │   ├── imu_corrector.param.yaml        # IMU correction settings
│   │   └── imu.param.yaml                  # General IMU configuration
│   ├── lidar/                  # LIDAR-specific configurations
│   │   ├── distortion_corrector_node.param.yaml  # LIDAR distortion correction
│   │   ├── passthrough_filter_node.param.yaml    # Point cloud filtering
│   │   ├── ring_outlier_filter.param.yaml       # Ring-based outlier removal
│   │   ├── velodyne_calibration.yaml            # Velodyne sensor calibration
│   │   ├── velodyne_driver.param.yaml           # Velodyne driver parameters
│   │   └── velodyne_transform.param.yaml        # Coordinate transformation
│   ├── localization/            # Localization algorithm configurations
│   │   ├── ekf_localizer.param.yaml        # Extended Kalman Filter parameters
│   │   ├── ndt_scan_matcher.param.yaml     # NDT scan matching configuration
│   │   └── pose_initializer.param.yaml     # Initial pose estimation
│   └── map/                     # Mapping-related configurations
│       ├── lanelet2_map_loader.param.yaml  # Lanelet2 map loading
│       ├── map_config.yaml                 # General map configuration
│       ├── map_projection_loader.param.yaml  # Map projection settings
│       ├── map_tf_generator.param.yaml     # TF (Transform) generation
│       └── pointcloud_map_loader.param.yaml  # Point cloud map loading
├── launch/                      # ROS2 launch files
│   ├── imu.launch.py            # IMU sensor launch configuration
│   ├── lidar_container31.launch.py  # LIDAR container launch
│   ├── lidar.launch.py          # LIDAR sensor launch
│   ├── lidar_perception.launch.py  # LIDAR perception pipeline
│   ├── localization.launch.py   # Localization system launch
│   └── map.launch.py            # Map loading and setup
├── package.xml                  # ROS2 package manifest
└── README.md                    # Project documentation
```

## Detailed File Explanations

### Build and Package Configuration

#### `CMakeLists.txt`
- Defines the build process for the ROS2 package
- Specifies dependencies, compilation rules, and build targets
- Configures how the package is built and installed

#### `package.xml`
- ROS2 package metadata
- Lists package dependencies
- Provides package information (maintainer, license, etc.)

### Configuration Files

#### IMU Configuration (`config/imu/`)
1. `gnss_poser.param.yaml
  - Configuration of `autoware` `gnss_poser` node. 
  - Nothing special here. Default values are used.

2. `imu_corrector.param.yaml`
  - Configuration of `autoware` `imu_corrector` node.
  - `stddev` values may be updated for better performance. Default values are used.

3. `imu.param.yaml`
  - Configuration of `SBG` imu&GNSS sensor.
  - Configured for `autoware` coordinate system and max data rate.
  - SBG has some performance issues, especially in GNSS and odometry data but I don't think it is related to this configuration.
  - Remember to update `leverArm` values if you change the gnss antenna position.


#### LIDAR Configuration (`config/lidar/`)
1. `distortion_corrector_node.param.yaml`
  - Configuration of `autoware` `distortion_corrector` node.
  - Nothing special here. Default values are used.

2. `passthrough_filter_node.param.yaml`
  - Configuration of `autoware` `passthrough_filter` node.
  - Lidar frames is set to `lidar_top`. If you change the frame, you should update this value.

3. `ring_outlier_filter.param.yaml`
  - Configuration of `autoware` `ring_outlier_filter` node.
  - Default values are used, only Lidar frame is set to `lidar_top`. If you change the frame, you should update this value.

4. `velodyne_calibration.yaml`
  - Velodyne factory calibration files.

5. `velodyne_driver.param.yaml`
  - Configuration of `velodyne` driver.
  - Ip adress and lidar frame is set. If you change the frame, you should update this value and if you are setting on a new device, you should update the ip adress.

6. `velodyne_transform.param.yaml`
  - Configuration for `velodyne_transform` node. Default values are used.

#### Localization Configuration (`config/localization/`)
1. `ekf_localizer.param.yaml`
   - Configuration for `autoware_ekf_localizer`.
  - Parameters can be tuned. I tested with different tf rates but it resulted with increased CPU usage and no performance gain. Other values can be updated I didn't have time to test them.

2. `ndt_scan_matcher.param.yaml`
  - Configuration for `autoware_ndt_scan_matcher`.
  - Some parameters are changed. I increased the iteration limit and increased epsilon value to make sure that the algorithm converges. Further tests can be done to optimize these values. I think you should focus on both ekf_localizer and ndt_scan_matcher configurations to get the best performance.

3. `pose_initializer.param.yaml`
  - Configuration for `autoware_pose_initializer`.
  - This is not used for now. The purpose of these packages to automatically initialize the pose of the vehicle. I tested automatic initialization for a while but I get 1 successful initialization in 3 tries. The main problem is the GPS data. GPS does not provide same coordinates every time. So I stop using automatic initialization and wrote a simple package to initialize pose manually. You can also find the package at [Pose Initializer](https://github.com/ITU-ZES-Solar-Car-Team/gae_initial_pose_adaptor). Use `v2` branch. Main is not completed.

#### Map Configuration (`config/map/`)
1. `map_config.yaml`
  - General map configuration
  - You have to change map path and lanelet path here. Do not change other files in this folder.


### Launch Files (`launch/`)
1. `imu.launch.py`
  - Launches imu related nodes including imu driver, imu_corrector and gnss_poser.
  - IMU driver provides SBG data.
  - IMU corrector is an autoware package to correct imu data.
  - GNSS poser is an autoware package to convert gnss data to pose data. This is not important for now. GNSS data is not used for localization now. You can consider removing if you don't get better IMU in the future.

2. `lidar_container31.launch.py`
  - Launches LIDAR related nodes including LIDAR driver, distortion corrector and voxel grid downsampler.
   - ROS2 component container is used to share the same LIDAR data with multiple nodes. It is very important for performance. If you implement any lidar based node in the future remember to add to the container.
  - Distortion corrector is an autoware package to compensate for LIDAR distortion.
  - Voxel grid downsampler, downsamples the point cloud data to reduce the computational load, very important for localization, otherwise localization will suck all the CPU power.

4. `lidar_perception.launch.py`
  - Launches LIDAR perception pipeline including ground segmentation and object clustering.
  - This package is set find objects that lies on the lanelet.
  - Ground segmentation applied on the full pointcloud and then lanelet2 filter is applied, finally clustering applied to those points.

5. `localization.launch.py`
    - Launches localization-related nodes including ndt_scan_matcher, ekf_localizer.
    - All topic are remapped to the where they belong. Topic names are self explanatory.

6. `map.launch.py`
    - Launches map related nodes.
    - Lanelet2 map and pointcloud map loaded here. You have to change the path of the maps in the configuration files.

## Prerequisites

- ROS2 (Recommended: Humble or Iron, maybe later versions)
- CMake
- Eigen3
- PCL (Point Cloud Library)
- Velodyne LIDAR drivers


## Usage

Launch the map and lanelet providers first:
```bash
ros2 launch gae_launch map.launch.py
```
Then run static tf publisher:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link imu_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link lidar_top
```

Launch IMU and Eagleye(optional, please find an encoder and stop using this):
```bash
ros2 launch gae_launch imu.launch.py
ros2 launch eagleye_rt eagleye_rt.launch.py
```

Finally launch localization and pose initializer:
```bash
ros2 launch gae_launch localization.launch.py
ros2 run gae_pose_initializer_v2 gae_pose_initializer_v2
```

## Acknowledgments
- This is not a completed project (sorry for that). There is lots of things to test and optimize. I tried to explain the main parts of the project. I hope this will help you to understand the project.
- Use an urdf instead of static tf publisher. I didn't have time to create an urdf for the vehicle. It is better and eventually you will need it for sensor fusion too.
- I don't like idea of using Eagleye for twist source but since we don't have an encoder and localization sometimes fail at rapid turns, I had to use it as a twist source. It needs to be initalized properly to work correctly. This is a temporary solution. Please find an encoder and stop using it.
- I didn't manage to use GNSS pose initialization because I always get non stable GNSS data. On every initialization I get different positions on same spot which is not good for pose initialization. 
- I also don't like to use autoware because while implementing localization, I need use lots of unnecessary packages and I have to deal with them. Consider to write a simpler package using libraries like `ndt_omp`. The idea is same but you can implement it in a more efficient way without dealing with autoware.


