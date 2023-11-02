import rclpy
from rclpy.node import Node


from tf2_ros import TransformBroadcaster
import numpy as np
import math
from math import atan2, pi, sin, cos, atan
from std_msgs.msg import UInt32
from std_msgs.msg import Float32  #TODO: need to change to standard message type later
from sensor_msgs.msg import Imu # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
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





# some global constant
neutralPWM=1376
maxPWM=1765
minPWM=992

initPWM=neutralPWM
distanceTolerance =0.3 #robot stop moving when within xxx meter of waypoint (consider the fact that robot does not stop immediately when setting servo back to neutral)

kp = 5
kd = 5
ki = 0

goal_x = 10
goal_y =0

initial_x = 0
initial_y = 0
angleToleranceDifference=5
class PidController(Node):
    def __init__(self):
        super().__init__("pid_turning_controller")
        
        
        # some variable
        self.latestGlobalOdom=None
        self.goal_heading_angle_in_enu:float = None
        self.angleOffError:float = 0
        self.previousError=0
        self.accumulateError = 0
        self.wheel_to_compensate:str="none"
        self.isWithinGoalDistance = False # True if the robot is within a certain distance of current waypoint
        self.left_servo_pwm=initPWM
        self.right_servo_pwm=initPWM
        self.compensateValue:float= 0
        self.angleApproximateSteadyCounter = 0
        
        
        
        # first, create a subscriber to odometry/global
        self.odometryGlobalSub=self.create_subscription(Odometry, "/odometry/global", self.globalOdometryCallback,10)
        self.left_wheel_pwm_publisher = self.create_publisher(UInt32, "/steering_left", 10)
        self.right_wheel_pwm_publisher = self.create_publisher(UInt32, "/steering_right", 10)
        
        
        #self.loop()
    def determineIfWithinGoalDistance(self, distance):
        if(distance < distanceTolerance):
            return True
        else:
            return False
    def determineError(self, odom:Odometry):
        current_angle = self.calculateEulerAngleFromOdometry(odom)
        # current_x = odom.pose.pose.position.x
        # current_y = odom.pose.pose.position.y
        

        # return math.sqrt(  (current_x-goal_x)**2 + (current_y- goal_y)**2 )
        error = (self.goal_heading_angle_in_enu-current_angle)
        if error > 180 or error < -180:
            self.get_logger().error("error is way to big {0}".format(error))
        return error
    
    def isOverReach(self, odom:Odometry):
        
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y
        

        r1 =  math.sqrt(  (current_x-initial_x)**2 + (current_y- initial_y)**2 )
        
        r2 = math.sqrt((goal_x-initial_x)**2 + (goal_y-initial_y) **2)
        if r2 < r1:
            return True
        else:
            return False
    def calculateEulerAngleFromOdometry(self, odom:Odometry):
        # angle are returned in -180 to 180 degree
        rpy = euler_from_quaternion(odom.pose.pose.orientation)
        angle = rpy[2]*(180/pi)
        # if(angle < 0):
        #     angle +=360
        return angle
    def determineWheelPwm(self, odom:Odometry):
        angle = self.calculateEulerAngleFromOdometry(odom)
        

        
        compensate_info = ""
        
        
        
        # # https://stackoverflow.com/questions/31418567/is-angle-to-the-left-or-right-of-second-angle
        # difference = angle-self.goal_heading_angle_in_enu
        # if(difference < -180.0):
        #     difference +=360
        # if(difference > 180.0):
        #     difference-=360
        
        
        # if self.goal_heading_angle_in_enu - angle == 0:
        #     self.left_servo_pwm = initPWM
        #     self.right_servo_pwm = initPWM
        #     compensate_info= "none"
        # if difference >0.0:
        #     self.left_servo_pwm = initPWM + self.compensateValue
        #     self.right_servo_pwm = initPWM
        #     compensate_info= "left" # since is moving to left, need to compensate left to move faster, to correct it back 
        # else:
        #     self.left_servo_pwm = initPWM
        #     self.right_servo_pwm = initPWM + self.compensateValue
        #     compensate_info= "right"
    
    
    
        if self.angleOffError == 0:
            self.left_servo_pwm = initPWM
            self.right_servo_pwm = initPWM
            compensate_info= "none"
        if self.angleOffError <0.0:
            self.left_servo_pwm = initPWM + abs(self.compensateValue)
            self.right_servo_pwm = initPWM
            compensate_info= "left" # since is moving to left, need to compensate left to move faster, to correct it back 
        else:
            self.left_servo_pwm = initPWM
            self.right_servo_pwm = initPWM + abs(self.compensateValue)
            compensate_info= "right"
        self.get_logger().info("ENU angle {0}, current angle {1}, compensate for {2}".format(self.goal_heading_angle_in_enu, angle, compensate_info))
    def globalOdometryCallback(self, odom:Odometry):
        
        if self.latestGlobalOdom == None or self.goal_heading_angle_in_enu == None:
            self.latestGlobalOdom= odom   
            
            # calculate the desire heading
            # dx =  goal_x - odom.pose.pose.position.x
            # dy =  goal_y - odom.pose.pose.position.y
            # initial_x = odom.pose.pose.position.x
            # initial_y=odom.pose.pose.position.y
            dx = goal_x - initial_x
            dy = goal_y-initial_y
            
            self.goal_heading_angle_in_enu = math.atan2(dy,dx)*(180/pi)  # this leaves it in  a negative degree
            if(self.goal_heading_angle_in_enu > 180 or self.goal_heading_angle_in_enu < -180):
                self.get_logger().error("Heading angle is not within the -180 to 180 range {0}".format(self.goal_heading_angle_in_enu))
            # if self.goal_heading_angle_in_enu < 0:
            #     self.goal_heading_angle_in_enu += 360
            
            self.get_logger().info("dx is {0} dy is {1}".format(dx,dy))
            self.get_logger().info("initial heading angle is {0}".format(self.goal_heading_angle_in_enu))
        else:
            # # calculate error   
            # # it is basically the euler formula
            # self.previousError = self.angleOffError # save previous error
            # self.angleOffError = self.determineError(odom)
            # self.accumulateError += self.angleOffError
            
            # see if the robot is actually already within the acceptable distance
            #self.determineIfWithinGoalDistance(self.distanceError)
            if abs(self.determineError(odom)) < angleToleranceDifference:
                self.angleApproximateSteadyCounter += 1
            else:
                self.angleApproximateSteadyCounter = 0
            if self.angleApproximateSteadyCounter >=100:
                self.isWithinGoalDistance = True
                self.get_logger().info("angle reached")
            else:
                # now start to compensate a value for it 
                # calculate error   
                # it is basically the euler formula
                self.previousError = self.angleOffError # save previous error
                self.angleOffError = self.determineError(odom)
                self.accumulateError += self.angleOffError  # see https://discuss.bluerobotics.com/t/using-pid-with-heading-control/223/4 , should be good as long as -180<= errorAngle <= 180
                self.compensateValue = self.pidCalculation()
                
                
                # if negative, means is at left, so compensate left by taking the absolute error of it
                # find which wheel to compensate
                self.wheel_to_compensate = self.determineWheelPwm(odom)
        self.loop()

         
    def pidCalculation(self):
        return kp * self.angleOffError + kd * (self.angleOffError-self.previousError) + ki*self.accumulateError
        
        
    def loop(self):
        
        if self.isWithinGoalDistance == False:
            
            # do the pid calculation
            # consider not I term? Since no negative error, all error is + due to pythagorean theorem, and also we cut all when robot within certain distance
            
            # publish out the pwm value 
            pass
        else:
            self.left_servo_pwm = neutralPWM
            self.right_servo_pwm = neutralPWM
        
        left_pwm = UInt32()
        right_pwm =UInt32()
        
        
        left_pwm.data= self.roundPwmValue(self.left_servo_pwm)
        right_pwm.data = self.roundPwmValue(self.right_servo_pwm)
        
        
        self.left_wheel_pwm_publisher.publish(left_pwm)
        self.right_wheel_pwm_publisher.publish(right_pwm)
        
            
        self.get_logger().info("left pwm {0} right pwm {1}".format(self.left_servo_pwm, self.right_servo_pwm))

    def roundPwmValue(self, value):
        if value > maxPWM:
            return maxPWM
        elif value < minPWM:
            return minPWM
        else:
            return round(value)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PidController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()