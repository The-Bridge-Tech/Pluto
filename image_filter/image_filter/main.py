import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes
from .extract_from_point_cloud import extra_pointcloud

class ImageFilter(Node):

    def __init__(self):
        super().__init__('image_filter')
        
        
        #TODO: check for the difference in frame
        #TODO: check for the lagging of time 
        
        # 1. subscribe the image that was published by the camera
        self.aligned_picture_subscription = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw",self.depth_aligned_image_callback,1
        )
        
        # 2. subscribe to the output from yolo model
        
        self.object_bounding_boxes_subscription = self.create_subscription(
            BoundingBoxes, "/bounding_boxes", self.object_bounding_boxes_callback,1
        )
        # 3. subscribe to the point-cloud ot the camera, 
            # since 
        self.pointclound_subscription = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self.image_pointcloud_callback,1
        )

        #5. publish the filtered result
        self.filtered_pointcloud_publisher = self.create_publisher(
            PointCloud2,"/filtered_pointcloud",1
        )
        timer_period = 0.5 # in second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.latest_aligned_image =None
        self.bounding_boxes_object = None
        self.image_pointcloud = None
        
        
        self.filtered_pointcloud = None
        
    def depth_aligned_image_callback(self, message:Image):
        self.latest_aligned_image = message

    def object_bounding_boxes_callback(self, message:BoundingBoxes):
        self.bounding_boxes_object = message
        
    def image_pointcloud_callback(self, message:PointCloud2):
        self.image_pointcloud = message
    
    def calculate_time_difference_of_message(self)->list:
        time_of_aligned_image = (self.latest_aligned_image.header.stamp.sec + self.latest_aligned_image.header.stamp.nanosec/(10**9))
        time_of_bounding_boxes = (self.bounding_boxes_object.header.stamp.sec + self.bounding_boxes_object.header.stamp.nanosec/(10**9))
        time_of_pointcloud = (self.image_pointcloud.header.stamp.sec + self.image_pointcloud.header.stamp.nanosec/(10**9))
        
        return [abs(time_of_aligned_image-time_of_bounding_boxes), abs(time_of_aligned_image-time_of_pointcloud), abs(time_of_bounding_boxes-time_of_pointcloud)]
    
    def is_message_within_time_tolerance(self, tolerance_second=0.5):
        
        if self.bounding_boxes_object is None or self.image_pointcloud is None or self.latest_aligned_image is None:
            return False
        else:
            for val in self.calculate_time_difference_of_message():
                if val > tolerance_second:
                    return False
            
            return True
            
            
            
    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # #self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        if self.is_message_within_time_tolerance():
            self.filtered_pointcloud = PointCloud2()
            self.filtered_pointcloud.header = self.image_pointcloud.header
            
            #height
            #width
            
            self.filtered_pointcloud.fields = self.image_pointcloud.fields
            
            self.filtered_pointcloud.is_bigendian = self.image_pointcloud.is_bigendian
            self.filtered_pointcloud.point_step = self.image_pointcloud.point_step
            #row_step
            #data
            self.filtered_pointcloud.is_dense = self.image_pointcloud.is_dense
            
            self.get_logger().info("start filter")
            extra_pointcloud(self.image_pointcloud, self.latest_aligned_image, self.bounding_boxes_object, self.filtered_pointcloud)
            
            self.filtered_pointcloud_publisher.publish(self.filtered_pointcloud)
            self.get_logger().info("Finished publish")
        else:
            self.get_logger().error("Message lagged out of tolerance 0.5 second time")
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImageFilter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()