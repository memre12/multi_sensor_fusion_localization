#!/usr/bin/env python3
"""
ZED2 Minimal Driver - Sol Kamera, Derinlik, PointCloud ve Odometri Yayınlar
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

import pyzed.sl as sl
import numpy as np
import struct


class ZedMinimalDriver(Node):
    def __init__(self):
        super().__init__('zed_minimal_driver')

        # Publishers
        self.pub_left = self.create_publisher(
            Image,
            '/zed2_left_camera/image_raw',  # SIM
            10
        )
        self.pub_depth = self.create_publisher(
            Image,
            '/zed2_left_camera/depth/image_raw',  # SIM
            10
        )
        self.pub_cam_info = self.create_publisher(
            CameraInfo,
            '/zed2_left_camera/camera_info',  # SIM
            10
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            '/zed2/odom',  # SIM
            10
        )

        # NEW: PointCloud publisher
        self.pub_cloud = self.create_publisher(
            PointCloud2,
            '/zed2/points',  # SIM
            10
        )

        # CV Bridge
        self.bridge = CvBridge()

        # ZED Kamera başlatma
        self.zed = sl.Camera()

        # Kamera parametreleri
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p (1280x720)
        init_params.camera_fps = 30  # 30 FPS
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_minimum_distance = 0.3
        init_params.depth_maximum_distance = 10.0

        # Kamerayı aç
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'ZED kamera açılamadı: {status}')
            rclpy.shutdown()
            return

        self.get_logger().info('ZED2 kamera başarıyla açıldı')

        # Positional Tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.set_as_static = False
        tracking_params.enable_imu_fusion = True

        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Positional tracking etkinleştirilemedi: {status}')
        else:
            self.get_logger().info('Positional tracking etkinleştirildi')

        # Kamera bilgileri
        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters
        left_cam = calibration_params.left_cam

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = 'zed_left_camera_optical_frame'
        self.camera_info_msg.width = self.zed.get_camera_information().camera_configuration.resolution.width
        self.camera_info_msg.height = self.zed.get_camera_information().camera_configuration.resolution.height

        self.camera_info_msg.distortion_model = 'rational_polynomial'
        self.camera_info_msg.d = [0.0] * 8  # rectified varsayımı

        self.camera_info_msg.k = [
            left_cam.fx, 0.0, left_cam.cx,
            0.0, left_cam.fy, left_cam.cy,
            0.0, 0.0, 1.0
        ]
        self.camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.camera_info_msg.p = [
            left_cam.fx, 0.0, left_cam.cx, 0.0,
            0.0, left_cam.fy, left_cam.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Runtime parametreleri
        self.runtime_params = sl.RuntimeParameters()

        # Mat buffers
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()

        # NEW: Point cloud buffer (XYZRGBA)
        self.cloud_mat = sl.Mat()

        # Pose
        self.zed_pose = sl.Pose()

        # Velocity calc
        self.prev_translation = None
        self.prev_orientation = None
        self.prev_time = None

        # Timer
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info('ZED2 minimal driver hazır! (Image+Depth+Cloud+Odom)')

    def timer_callback(self):
        """Her 33ms'de bir görüntü/derinlik/cloud al ve yayınla (30 FPS)"""
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        stamp = self.get_clock().now().to_msg()

        # Left image
        self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
        left_cv = self.left_image.get_data()[:, :, :3]  # BGRA -> BGR

        # Depth
        self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
        depth_cv = self.depth_image.get_data()

        # Tracking pose
        tracking_state = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)

        # Point cloud (XYZRGBA)
        # Not: PERFORMANCE için resolution’da point cloud ağır olabilir, ama HD720 genelde OK.
        self.zed.retrieve_measure(self.cloud_mat, sl.MEASURE.XYZRGBA)
        cloud_np = self.cloud_mat.get_data()  # shape: (H,W,4) float32 (x,y,z,rgba_as_float)

        try:
            # Publish left
            left_msg = self.bridge.cv2_to_imgmsg(left_cv, encoding='bgr8')
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = 'zed_left_camera_optical_frame'
            self.pub_left.publish(left_msg)

            # Publish depth
            depth_msg = self.bridge.cv2_to_imgmsg(depth_cv, encoding='32FC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'zed_left_camera_optical_frame'
            self.pub_depth.publish(depth_msg)

            # Publish CameraInfo
            self.camera_info_msg.header.stamp = stamp
            self.pub_cam_info.publish(self.camera_info_msg)

            # Publish cloud
            cloud_msg = self.numpy_xyzrgba_to_pointcloud2(
                cloud_np,
                frame_id='zed_left_camera_optical_frame',
                stamp=stamp
            )
            self.pub_cloud.publish(cloud_msg)

            # Publish odom
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                self.publish_odometry(stamp)

        except Exception as e:
            self.get_logger().error(f'Publish/dönüştürme hatası: {e}')

    def numpy_xyzrgba_to_pointcloud2(self, cloud_np: np.ndarray, frame_id: str, stamp):
        """
        cloud_np: (H, W, 4) float32 => x,y,z,rgba(float32, packed)
        Output: PointCloud2 with fields x,y,z,rgba (uint32)
        """
        h, w, c = cloud_np.shape
        assert c == 4

        # Flatten to Nx4
        flat = cloud_np.reshape(-1, 4)

        # Filter invalid (NaN / Inf) points (ZED invalids often NaN)
        xyz = flat[:, :3]
        valid = np.isfinite(xyz).all(axis=1)
        flat = flat[valid]

        # rgba packed is float32 that contains 4x uint8 packed inside; reinterpret bits to uint32
        rgba_float = flat[:, 3].astype(np.float32)
        rgba_u32 = rgba_float.view(np.uint32)

        # Pack bytes: x,y,z float32 + rgba uint32
        # Create structured array for efficient tobytes()
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgba', np.uint32),
        ])
        pc = np.empty(flat.shape[0], dtype=dtype)
        pc['x'] = flat[:, 0].astype(np.float32)
        pc['y'] = flat[:, 1].astype(np.float32)
        pc['z'] = flat[:, 2].astype(np.float32)
        pc['rgba'] = rgba_u32

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = pc.shape[0]

        msg.is_bigendian = False
        msg.is_dense = False  # invalidler filtrelendi ama yine de False bırakmak güvenli

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        msg.data = pc.tobytes()

        return msg

    def publish_odometry(self, stamp):
        translation = self.zed_pose.get_translation().get()
        orientation = self.zed_pose.get_orientation().get()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = float(translation[0])
        odom_msg.pose.pose.position.y = float(translation[1])
        odom_msg.pose.pose.position.z = float(translation[2])

        odom_msg.pose.pose.orientation.x = float(orientation[0])
        odom_msg.pose.pose.orientation.y = float(orientation[1])
        odom_msg.pose.pose.orientation.z = float(orientation[2])
        odom_msg.pose.pose.orientation.w = float(orientation[3])

        current_time = self.get_clock().now()
        if self.prev_translation is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                odom_msg.twist.twist.linear.x = (translation[0] - self.prev_translation[0]) / dt
                odom_msg.twist.twist.linear.y = (translation[1] - self.prev_translation[1]) / dt
                odom_msg.twist.twist.linear.z = (translation[2] - self.prev_translation[2]) / dt

                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0
        else:
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

        self.prev_translation = translation
        self.prev_orientation = orientation
        self.prev_time = current_time

        odom_msg.pose.covariance = [0.01] * 36
        odom_msg.twist.covariance = [0.01] * 36

        self.pub_odom.publish(odom_msg)

    def destroy_node(self):
        if hasattr(self, 'zed'):
            self.zed.close()
            self.get_logger().info('ZED kamera kapatıldı')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ZedMinimalDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
