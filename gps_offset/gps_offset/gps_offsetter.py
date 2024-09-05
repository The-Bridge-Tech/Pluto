"""
Calculate initial gps error and compensate further readings with an offset
Author: Matthew Lauriault
Created: 9/5/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

# CALCULATION MODULES
from pyproj import Proj, transform


# CONSTANTS
BASE_GPS = (34.841400, -82.411743)


class GPSOffsetter(Node):

        def __init__(self):

                super().__init__('gps_offsetter')

                # SUBSCRIBERS
                # Subscribe to gps topic to get initial gps data
                self.gps_subscriber = self.create_subscription(
                        NavSatFix, 
                        "/fix", 
                        self.gps_callback, 
                        10
                )
                self.initialGPS = None
                self.currentGPS = None
                # Subscribe to autonomous mode topic to wait for autonomous mode to start
                self.is_autonomous_mode_sub = self.create_subscription(
                        Bool, 
                        "is_autonomous_mode", 
                        self.is_autonomous_mode_callback, 
                        10
                )
                self.is_autonomous_mode = False

                # PUBLISHERS
                # Publish gps message with the added offset
                self.offset_gps_publisher = self.create_publisher(
                        NavSatFix,
                        "/fix/offset",
                        10
                )

                # VARIABLES
                self.easting_offset = None
                self.northing_offset = None

                # Projection setup: WGS84 to UTM
                self.proj_wgs84 = Proj(proj='latlong', datum='WGS84')
                self.proj_utm = Proj(proj='utm', zone=17, datum='WGS84')  # Update zone as per your location
                # Convert base point to UTM coordinates
                self.base_easting, self.base_northing = self.getUTM(*BASE_GPS)

        
        # HELPERS

        def getUTM(self, lat: float, lon: float) -> tuple:
                return transform(
                        self.proj_wgs84, 
                        self.proj_utm, 
                        lon,
                        lat
                )
        
        def getLatLon(self, easting: float, northing: float) -> tuple:
                return transform(
                        self.proj_utm,
                        self.proj_wgs84,
                        easting,
                        northing
                )
        
        def calculateOffset(self):
                """Calculate offset error in initial GPS reading."""
                # Convert initial GPS reading to UTM coordinates
                initial_easting, initial_northing = self.getUTM(
                        lat = self.initialGPS.latitude,
                        lon = self.initialGPS.longitude
                )
                # Calculate the offset in UTM coordinates
                self.easting_offset = self.base_easting - initial_easting
                self.northing_offset = self.base_northing - initial_northing

                self.get_logger().info(f"Calculated offset: ({self.easting_offset}, {self.northing_offset})")

        def applyOffset(self):
                """Apply offset to current GPS"""
                # Convert current GPS reading to UTM coordinates
                current_easting, current_northing = self.getUTM(
                        lat = self.currentGPS.latitude,
                        lon = self.currentGPS.longitude
                )
                # Compensate the offset in UTM
                corrected_easting = current_easting + self.easting_offset
                corrected_northing = current_northing + self.northing_offset
                # Convert the corrected UTM back to lat and lon
                corrected_lon, corrected_lat = self.getLatLon(
                        easting = corrected_easting,
                        northing = corrected_northing
                )
                # Apply the corrected lat and lon to the current GPS message
                self.currentGPS.latitude = corrected_lat
                self.currentGPS.longitude = corrected_lon
                self.get_logger().info(f"Corrected GPS: ({self.currentGPS.latitude }, {self.currentGPS.longitude})")


        # SUBSCRIBER CALLBACKS

        def gps_callback(self, msg: NavSatFix):
                """Update initial and current GPS."""
                # Wait for autonous mode -> then get initial gps
                if self.is_autonomous_mode:
                        if not self.initialGPS:
                                self.initialGPS = msg
                                self.calculateOffset()
                        self.currentGPS = msg
                        self.applyOffset()
                        self.offset_gps_publisher.publish(self.currentGPS)
                else:
                        self.get_logger().info("Waiting for autonomous mode...")
                        

        def is_autonomous_mode_callback(self, msg: Bool):
                """Update if in autonomous mode."""
                self.is_autonomous_mode = msg.data


# MAIN

def main(args=None):
        rclpy.init(args=args)
        gps_offsetter = GPSOffsetter()

        rclpy.spin(gps_offsetter)

        gps_offsetter.destroy_node()
        rclpy.shutdown()


# When this file is run as a script
if __name__ == '__main__':
        main()