

# create a new point cloud message that is 
# let it be unorganized, but densed(base on the type from the original point cloud message)

import cv2
import numpy as np
# import matplotlib.pyplot as plt
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox
from cv2 import Mat

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
import os

from cv_bridge import CvBridge

def create_blank(width, height, rgb_color=(0, 0, 0)):
    """Create new image(numpy array) filled with certain color in RGB"""
    # Create black blank image
    image = np.zeros((height, width, 3), np.uint8)

    # Since OpenCV uses BGR, convert the color first
    color = tuple(reversed(rgb_color))
    # Fill image with color
    image[:] = color

    return image


# image_path = os.path.join(os.getcwd(), "yoloxTest.png")
# # print(image_path)
# img:Mat = cv2.imread(image_path)
# # print(img)
# # cv2.imshow("original",img)
# # cv2.waitKey(0)


# height, width, depth = img.shape
# print(img.shape)

# new_image = create_blank(width, height)

# # cv2.imshow('black',new_image)
# print(img[2,4])
# # print(new_image)

# # cv2.destroyAllWindows()

# box1 = BoundingBox()
# box1.probability=0.805
# box1.xmin=44
# box1.xmax=600
# box1.ymin=93
# box1.ymax=482
# box1.img_width=640
# box1.img_height=480

box2 = BoundingBox()
box2.probability=0.645
box2.xmin=287
box2.ymin=263
box2.xmax=384
box2.ymax=413
box2.img_width=640
box2.img_height=480
test_bounding_message = BoundingBoxes()
test_bounding_message.bounding_boxes = [box2]





def extra_object(image:Mat, bounding_message:BoundingBoxes, new_image:Mat):
    
    
    for box_message in bounding_message.bounding_boxes:
        
        box_info:BoundingBox = box_message
        
        
        for i in range(box_info.ymin-1,min(box_info.ymax, box_info.img_height)):
            
            print("y " + str(i) + "max " + str( min(box_info.ymax, box_info.img_width)))
            for k in range(box_info.xmin-1, box_info.xmax):
                #new_image[k,i] = image[k,i]
                new_image[i,k]= image[i,k]
                print(k)
def extra_pointcloud(original_pointcloud:PointCloud2, original_picture:Image, bounding_message:BoundingBoxes, new_pointcloud:PointCloud2):
    
    

    # assume a one-to-one match between original_pointcloud and original_picture
    number_of_point = 0
    for box_message in bounding_message.bounding_boxes:
        
        box_info:BoundingBox = box_message
        
        
        for i in range( max(0,box_info.ymin-1),min(box_info.ymax, original_picture.height)):

            for k in range( max(0, box_info.xmin-1), min(box_info.xmax,original_picture.width)):
                # assume pointcloud2 is organized
                
                # assume each point contain x,y,z,depth, color
                start_index = i*original_pointcloud.point_step*original_pointcloud.width + k*original_pointcloud.point_step
                end_index = start_index + original_pointcloud.point_step
                
                #print( f"i {i} k{k} point_step {original_pointcloud.point_step} Start index {start_index}  endindex {end_index}".format(i,k, original_pointcloud.point_step,start_index, end_index))
                valueSize = original_pointcloud.point_step/ len(original_pointcloud.fields)
                new_pointcloud.data = new_pointcloud.data + original_pointcloud.data[start_index:end_index]
                
                # for data_size in range(start_index, end_index, valueSize):
                    
                # i*20*number_in_row + k*20
                
                number_of_point+=1
       

    
    new_pointcloud.height=1
    new_pointcloud.width=number_of_point
    new_pointcloud.row_step = new_pointcloud.point_step* number_of_point
    
    
    # # for debugging purpose
    # # Create an iterator over the points in the message
    # iterator = pc2.read_points(msg)

    # # Extract the x, y, and z coordinates from the points
    # points = np.array(list(iterator))
    # # Extract the x, y, and z coordinates from the points array
    # x = points[:, 0]
    # y = points[:, 1]
    # z = points[:, 2]
    # height = new_pointcloud.height
    # width = new_pointcloud.width
    # x = x.reshape(height, width)
    # y = y.reshape(height, width)
    # z = z.reshape(height, width)
    # color_image = np.zeros((height, width, 3), dtype=np.uint8)
    # color_image[:, :, 0] = (x / np.max(x)) * 255
    # color_image[:, :, 1] = (y / np.max(y)) * 255
    

    # new_image = create_blank(width, height)
    # bridge = CvBridge()
    # img_rgb = bridge.imgmsg_to_cv2(original_picture,"bgr8")
    
    
    # extra_object(img_rgb, bounding_message, new_image)
    
    # cv2.imwrite("pointCloud_filtered_result.png", color_image)
    # cv2.imwrite("picture_filter_result.png", new_image)
    
# extra_object(img, bounding_message, new_image)
# cv2.imshow("original",new_image)
# cv2.waitKey(0)
