# NtripDriver - ROS 2 Node

The NtripDriver ROS 2 node facilitates communication with an NTRIP (Networked Transport of RTCM via Internet Protocol) server, handling RTCM messages and serial port data transmission. This node is designed to work within the ROS 2 ecosystem.

#### Author: [Robeff Technology](https://www.robeff.com)
#### Maintainer : [Robeff Technology](mailto:support@robeff.com)

## Features

- Establishes communication with an NTRIP server.
- Handles RTCM messages received from the NTRIP server.
- Optionally publishes RTCM messages and/or transmits them over a serial port.
- Supports initialization of the NTRIP client's location using NavSatFix messages.
- Configurable parameters for NTRIP server connection, serial port settings, and RTCM message publication.

## Installation

1. Clone this repository into your Humble workspace:

    ```bash
    git clone https://github.com/Robeff-Technology/rbf_ntrip_driver.git
    ```

2. Install dependencies using ```rosdep```:
   ```bash
   cd rbf_ntrip_driver
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the Humble workspace:

    ```bash
    colcon build
    ```

## Usage

1. Configure the node's parameters by editing the YAML file located at `<path_to_repository>/config/rbf_ntrip_driver.param.yaml`. Adjust the parameters according to your setup.

2. Launch the ROS 2 node using the provided launch file:

    ```bash
    ros2 launch rbf_ntrip_driver rbf_ntrip_driver.launch.py
    ```

## Configuration

You can customize the behavior of the NtripDriver node by modifying the parameters in the `rbf_ntrip_driver.param.yaml` file. Refer to the documentation within the file for detailed explanations of each parameter.
### NTRIP Parameters:

1. **`host`**: 
   - The IP address or domain name of the NTRIP server.

2. **`user_name`**: 
   - The username for authentication with the NTRIP server.

3. **`password`**: 
   - The password for authentication with the NTRIP server.

4. **`mount_point`**: 
   - The mount point on the NTRIP server.

5. **`port`**: 
   - The port number used for communication with the NTRIP server.

6. **`use_nav_sat_fix_init`**: 
   - A boolean flag indicating whether to initialize the NTRIP client's location using NavSatFix messages.

7. **`nav_sat_fix_topic_name`**: 
   - The topic name where NavSatFix messages are published.

8. **`init_ntrip_location_lat`**: 
   - The initial latitude position for the NTRIP client.

9. **`init_ntrip_location_lon`**: 
   - The initial longitude position for the NTRIP client.

### Serial Port Parameters:

1. **`publish_port_rtcm`**: 
   - A boolean flag indicating whether to publish RTCM messages over the serial port.

2. **`name`**: 
   - The name of the serial port device.

3. **`baud_rate`**: 
   - The baud rate for serial communication.

### RTCM Publisher Parameters:

1. **`publish_rtcm`**: 
   - A boolean flag indicating whether to publish RTCM messages.

2. **`rtcm_topic`**: 
   - The topic name where RTCM messages are published.

3. **`rtcm_frame_id`**: 
   - The frame ID associated with RTCM messages.

These parameters allow you to configure various aspects of the NtripDriver node's behavior, including NTRIP server connection details, serial port settings, and RTCM message publication options. Adjusting these parameters enables customization to suit your specific application requirements.

## Node Details
- **Published Topics**:
    - `/rtcm` (mavros_msgs/RTCM): Published if `publish_rtcm` parameter is set to `true`.
- **Subscribed Topics**:
    - `/fix` (sensor_msgs/NavSatFix): Subscribed if `use_nav_sat_fix_init` parameter is set to `true`.


