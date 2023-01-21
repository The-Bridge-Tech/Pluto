
# from Phidget22.Phidget import *
# from Phidget22.Devices.Accelerometer import *
# from Phidget22.Devices.Gyroscope import *
# from Phidget22.Devices.Magnetometer import *
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
import numpy as np
import math
from math import atan2, pi, sin, cos, atan

from std_msgs.msg import Float32  #TODO: need to change to standard message type later
from sensor_msgs.msg import Imu # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
from geometry_msgs.msg import Vector3Stamped


#TODO: load the following information from a configuration file

# This node will publish a standard IMU message in ros

#TODO: need to adjust the heading later, due to ros requirement
# East is 0 degree, the x-axis    North is the y- axis
# https://www.ros.org/reps/rep-0103.html
"""
By the right hand rule, the yaw component of orientation increases as the child frame rotates counter-clockwise, and for geographic poses, yaw is zero when pointing east.

This requires special mention only because it differs from a traditional compass bearing, which is zero when pointing north and increments clockwise. Hardware drivers should make the appropriate transformations before publishing standard ROS messages.
"""
#TODO: does the imu publish in row, pitch, yaw? OR in euler's angle
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

# lastAnglesMag = [0,0,0]
# def onMagneticFieldChange(self, magneticField, timestamp):   #TODO, can remove the self here?
#     global lastAnglesMag
#     try:
#         lastAnglesMag[0] = magneticField[0]
#         lastAnglesMag[1] = magneticField[1]
#         lastAnglesMag[2] = magneticField[2]
#     except:
#         pass

# lastLinearAccel = [0, 0, 0]
# def onAccelerationChange(self, acceleration, timestamp):
#     global lastLinearAccel
#     try:
#         # rospy.logerr("Acceleration: " + str(acceleration))
#         lastLinearAccel[0] = acceleration[0]
#         lastLinearAccel[1] = acceleration[1]
#         lastLinearAccel[2] = acceleration[2]
#     except:
#         pass

# lastAngularAccel = [0,0,0]  #TODO: it is in row, pitch yaw?
# def onAngularAccel(self, acceleration, timestamp):
#     global lastAngularAccel
#     try:
#         # rospy.logerr("AngularAccel: " + str(acceleration))
#         lastAngularAccel[0] = acceleration[0]
#         lastAngularAccel[1] = acceleration[1]
#         lastAngularAccel[2] = acceleration[2]
#     except:
#         pass

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
class CompassPublisher(Node):
    def __init__(self):

        super().__init__("compass_publisher")

    
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_heading)
        self.heading_publisher = self.create_publisher(Float32, 'heading_message',10)    

        
        self.imu_listener = self.create_subscription(
            Imu,
            'imu/data',
            self.convert_quaternion_to_euler_angle,
            10)
        
        self.imu_debug_listener = self.create_subscription(
            Vector3Stamped, '/imu/rpy/filtered', self.publish_debug_heading,10
            
            
        )

        self.yaw_angle = Float32()
    def publish_debug_heading(self, message:Vector3Stamped):
        
        result = ( message.vector.z * 3.14 )/180
        print( "Debug heading is " + str(self.yaw_angle.data))    
    def publish_heading(self):
        self.heading_publisher.publish(self.yaw_angle)
        print( "heading is " + str(self.yaw_angle.data))
    def convert_quaternion_to_euler_angle(self, imu_message:Imu):
        xyzw :list = imu_message.orientation
        
        xyz = euler_from_quaternion(xyzw)
        self.yaw_angle.data = xyz[2]*(180/pi)
        
        
        
        
    # def calculate_heading(self) -> list[float, float, float]:
    #     """
    #     calculate_heading _summary_

    #     _extended_summary_
        
    #     Note:
    #     ------
    #     This is the old code from ros 1 developement.

    #     Returns
    #     -------
    #     float
    #         Heading in degree
    #     """
        
    #     #TODO: is this related?
    #     #https://math.stackexchange.com/questions/2466949/get-magnetic-field-values-from-euler-angle
    #     global lastAnglesMag
        
    #     rollAngle = atan2(lastLinearAccel[1], lastLinearAccel[2])
    #     pitchAngle = atan(-lastLinearAccel[0] / (lastLinearAccel[1]
    #                       * sin(rollAngle) + lastLinearAccel[2] * cos(rollAngle)))
    #     atanarg1 = lastAnglesMag[2] * \
    #         sin(rollAngle) - lastAnglesMag[1] * cos(rollAngle)
    #     atanarg2 = lastAnglesMag[0] * cos(pitchAngle) + lastAnglesMag[1] * sin(
    #         pitchAngle) * sin(rollAngle) + lastAnglesMag[2] * sin(pitchAngle) * cos(rollAngle)
    #     yawAngle = atan2(atanarg1, atanarg2)
    #     compassBearing = yawAngle * (180.0 / pi)
        
    #     # rospy.logerr("MagneticField: \t"+ str(magneticField[0]*100)+ "  |  "+ str(magneticField[1]*100)+ "  |  "+ str(magneticField[2]*100))
    #     # rospy.logerr("MagneticField: \t"+ str(compassBearing + 180))

    #     # angles = [ rollAngle, pitchAngle, yawAngle ]

    #     # try:
    #     #     i = 0
    #     #     for i in range(3):
    #     #         if abs(angles[i] - lastAngles[i]):
    #     #             for value in compassBearing
    #     # except:
    #     #     pass

    #     # i think this is the formula for determining the direction of north based on the magnetic field vector being read in
    #     # heading = atan2(magneticField[1], magneticField[0]) * 180 / (pi + 180)

    #     # the compass currently reads in reverse, so correcting that (90 on compass is 270 irl)
    #     # this is just due to the direction the compass is facing, i have the wire in the back
    #     real_heading = (compassBearing + 90)
    #     if real_heading < 0:
    #         real_heading += 360

    #     # generic information for debugging this program
    #     # print("My Calculation: " + str(real_heading))
    #     # rospy.logerr("MagneticField: \t"+ str(magneticField[0]*100)+ "  |  "+ str(magneticField[1]*100)+ "  |  "+ str(magneticField[2]*100))
    #     # print("Timestamp: " + str(timestamp)) 
        
    #     self.heading_publisher.publish(real_heading)
    #     return rollAngle, pitchAngle, yawAngle
    #     # print("----------")
        
    # def imu_callback(self):
    #     """
    #     This function publish IMU message to /compass_pulisher.

    #     _extended_summary_
    #     """
    #     # wiki.ros.org/rospy/Overview/Timea
    #     #https://answers.ros.org/question/189867/what-is-the-timestamp/
    #     # https://answers.ros.org/question/385771/header-and-timestamp-for-float32multiarray-in-ros2/?answer=385799
    #     self.imu_message.header.frame_id = frame_id
    #     self.imu_message.header.stamp =  self.get_clock().now().to_msg()
    #     # note: do not write to seq in header
    #     # https://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
        
    #     # 1. get the heading
    #     #1.1 publish to heading_messge/, add to imu_message
    #     row_pitch_yaw = self.calculate_heading()  #TODO: are they in radian?
        
    #     q = quaternion_from_euler(row_pitch_yaw[0], row_pitch_yaw[1], row_pitch_yaw[2])
    #     self.imu_message.orientation.x = q[0]
    #     self.imu_message.orientation.y = q[1]
    #     self.imu_message.orientation.z = q[2]
    #     self.imu_message.orientation.w = q[3]
        
    #     # setting up the quaternion covariance
    #     #TODO what should the covariance here? Right now is 0.1 radians
    #     self.imu_message.orientation_covariance = [0.1,0,0,
    #                                                0,0.1,0,
    #                                                0,0,0.1]

    #     #2. get the angular acceleration
    #     #TODO: is this in correct order?
    #     self.imu_message.angular_velocity.x = lastAngularAccel[0]
    #     self.imu_message.angular_velocity.y = lastAngularAccel[1]
    #     self.imu_message.angular_velocity.z = lastAngularAccel[2]
        
    #     #TODO: need to update the covariance
    #     self.imu_message.angular_velocity_covariance = [0.1, 0, 0,
    #                                                     0, 0.1, 0,
    #                                                     0, 0, 0.1]
        
    #     # 3. get the linear acceleration
    #     self.imu_message.linear_acceleration.x = lastLinearAccel[0]
    #     self.imu_message.linear_acceleration.y = lastLinearAccel[1]
    #     self.imu_message.linear_acceleration.z = lastLinearAccel[2]
    #     #TODO: is this covariance correct? 0.1 m/s^2?
    #     self.imu_message.linear_acceleration_covariance = [0.1, 0, 0,
    #                                                        0, 0.1, 0,
    #                                                        0, 0, 0.1]


# # lastAngles = [0, 0, 0]
# lastAnglesMag = [0, 0, 0]


# def onMagneticFieldChange(self, magneticField, timestamp):
#     global lastAnglesMag
#     try:
#         lastAnglesMag[0] = magneticField[0]
#         lastAnglesMag[1] = magneticField[1]
#         lastAnglesMag[2] = magneticField[2]

#         rollAngle = atan2(lastAnglesAccel[1], lastAnglesAccel[2])
#         pitchAngle = atan(-lastAnglesAccel[0] / (lastAnglesAccel[1]
#                           * sin(rollAngle) + lastAnglesAccel[2] * cos(rollAngle)))
#         atanarg1 = lastAnglesMag[2] * \
#             sin(rollAngle) - lastAnglesMag[1] * cos(rollAngle)
#         atanarg2 = lastAnglesMag[0] * cos(pitchAngle) + lastAnglesMag[1] * sin(
#             pitchAngle) * sin(rollAngle) + lastAnglesMag[2] * sin(pitchAngle) * cos(rollAngle)
#         yawAngle = atan2(atanarg1, atanarg2)
#         compassBearing = yawAngle * (180.0 / pi)
#     except:
#         pass
#     # rospy.logerr("MagneticField: \t"+ str(magneticField[0]*100)+ "  |  "+ str(magneticField[1]*100)+ "  |  "+ str(magneticField[2]*100))
#     # rospy.logerr("MagneticField: \t"+ str(compassBearing + 180))

#     # angles = [ rollAngle, pitchAngle, yawAngle ]

#     # try:
#     #     i = 0
#     #     for i in range(3):
#     #         if abs(angles[i] - lastAngles[i]):
#     #             for value in compassBearing
#     # except:
#     #     pass

#     # i think this is the formula for determining the direction of north based on the magnetic field vector being read in
#     # heading = atan2(magneticField[1], magneticField[0]) * 180 / (pi + 180)

#     # the compass currently reads in reverse, so correcting that (90 on compass is 270 irl)
#     # this is just due to the direction the compass is facing, i have the wire in the back
#     real_heading = (compassBearing + 90)
#     if real_heading < 0:
#         real_heading += 360

#     # generic information for debugging this program
#     # print("My Calculation: " + str(real_heading))
#     # rospy.logerr("MagneticField: \t"+ str(magneticField[0]*100)+ "  |  "+ str(magneticField[1]*100)+ "  |  "+ str(magneticField[2]*100))
#     # print("Timestamp: " + str(timestamp))
#     compass_pub.publish(real_heading)
#     # print("----------")



def main(args=None):
    rclpy.init(args=args)
    compass_publisher = CompassPublisher()
    rclpy.spin(compass_publisher)
    
    compass_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    # ch = Accelerometer()

    # # Register for event before calling open
    # ch.setOnAccelerationChangeHandler(onAccelerationChange)
    # ch.open()


    # gryoscope0 = Gyroscope()
    # gryoscope0.setOnAngularRateUpdateHandler(onAngularAccel)
    # gryoscope0.open()



    # # the following lines just establish the basic connection using the Phidget22 library
    # magnetometer0 = Magnetometer()
    # magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
    # magnetometer0.openWaitForAttachment(5000)
    
    main()

