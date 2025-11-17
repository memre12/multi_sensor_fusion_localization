#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker

import navsat_visul
from geocon import Geocon

class EkfToLatLon(Node):
    def __init__(self):
        super().__init__('ekf_to_latlon')

        # Subscribers
        self.create_subscription(
            NavSatFix, '/imu/nav_sat_fix_corrected', self.gnss_callback, 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/ekf_pose_with_covariance', self.ekf_callback, 10)

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/ekf_latlon', 10)
        self.pub_marker = self.create_publisher(Marker, '/ekf_marker', 10)

        # Geocon
        self.geo = Geocon(0)

        # NATO UTM parameters
        self.latZone = None
        self.lngZone = None
        self.digraph = None

        self.gnss_ready = False

        self.get_logger().info("📡 EKF (NATO UTM) → Lat/Lon node started.")

    def gnss_callback(self, msg):
        utm = self.geo.latLngToUtm(msg.latitude, msg.longitude)
        
        nato = utm["nato"]

        # NATO GRID PARAM
        self.lngZone = nato["lngZone"]      # UTM Zone  (35)
        self.latZone = nato["latZone"]      # Latitude band (T)
        self.digraph = nato["digraph"]      # Grid digraph (PF)

        self.get_logger().info(
            f"GNSS REF → Zone={self.lngZone}{self.latZone}, Digraph={self.digraph}"
        )

        self.gnss_ready = True

    def ekf_callback(self, msg):
        if not self.gnss_ready:
            self.get_logger().warn("Waiting for GNSS reference...")
            return

        # EKF NATO UTM easting/northing
        e = float(msg.pose.pose.position.x)
        n = float(msg.pose.pose.position.y)
        alt = float(msg.pose.pose.position.z)

        # Convert NATO → Lat/Lon
        latlon = self.geo.natoToLatLng(
            e,
            n,
            self.lngZone,   # lng zone (35)
            self.latZone,   # lat zone (T)
            self.digraph    # digraph (PF)
        )

        lat = latlon["lat"]
        lon = latlon["lng"]

        fix = NavSatFix()
        fix.header = msg.header
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        self.pub_fix.publish(fix)

        # RViz Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.pose.position.x = e
        marker.pose.position.y = n
        marker.pose.position.z = alt
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.pub_marker.publish(marker)

        self.get_logger().info(f"EKF → lat={lat:.6f}, lon={lon:.6f}")


def main():
    rclpy.init()
    node = EkfToLatLon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
