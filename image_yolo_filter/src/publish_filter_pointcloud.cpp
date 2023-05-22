

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/time_sequencer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/cache.h"
// #include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include "publish_filter_pointcloud.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <fstream>
#include <limits>
#include <vector>
using namespace std::chrono_literals;
// using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;

// https://answers.ros.org/question/234455/pointcloud2-and-pointfield/#234472

void ImageYoloFilter::convertToPng(const sensor_msgs::msg::Image& image_msg, const std::string& filename)
{
  // Convert the Image message to an OpenCV Mat object
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  // Create a PNG image buffer
  std::vector<uchar> png_buffer;
  cv::imencode(".png", image, png_buffer);

  // Write the PNG image buffer to a file
  std::ofstream file(filename, std::ios::binary);
  if (file.is_open())
  {
    file.write(reinterpret_cast<const char*>(png_buffer.data()), png_buffer.size());
    file.close();
  }
  else
  {
    throw std::runtime_error("Failed to open file for writing: " + filename);
  }
}

ImageYoloFilter::ImageYoloFilter() : Node("ImageYoloFilter"), count_(0), qosCount(10)
{
  this->declare_parameter("debug", false);
  this->declare_parameter("timeLagToTolerance", 0.1);
  this->declare_parameter("cacheSize",15);
  // lag tolerance is 100 ms
  pointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
  imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/filtered_image", 10);
  // timer_ = this->create_wall_timer(50ms, std::bind(&ImageYoloFilter::timer_callback, this));

  // alignedPictureSubscription.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
  alignedPictureSubscription.subscribe(this, "/camera/color/image_raw");
  // //alignedPictureSubscription= message_filters::Subscriber<sensor_msgs::msg::Image>(this,
  // "/camera/aligned_depth_to_color/image_raw");
  // // alignedPictureSubscription = this->create_subscription<sensor_msgs::msg::Image>(
  // //   "/camera/aligned_depth_to_color/image_raw",10,std::bind(&ImageYoloFilter::alignedPictureCallback,this, _1));

  boundingBoxesSubscription.subscribe(this, "/bounding_boxes");
  // boundingBoxesSubscription = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
  //   "/bounding_boxes", 10, std::bind(&ImageYoloFilter::boundingBoxCallback,this, _1));

  alignedPointcloudSubscription.subscribe(this, "/camera/depth/color/points");
  // alignedPointcloudSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "/camera/depth/color/points",10, std::bind(&ImageYoloFilter::alignedPointcloudCallback,this, _1)
  // );


  
  this->cacheSize = this->get_parameter("cacheSize").get_parameter_value().get<int>();
  RCLCPP_INFO(this->get_logger(), "cache %d", this->cacheSize);

  this->imageCache.reset(new message_filters::Cache<sensor_msgs::msg::Image>(this->alignedPictureSubscription, this->cacheSize));
  this->pointcloudCache.reset(
      new message_filters::Cache<sensor_msgs::msg::PointCloud2>(this->alignedPointcloudSubscription, this->cacheSize));
  this->boundingBoxesCache.reset(
      new message_filters::Cache<bboxes_ex_msgs::msg::BoundingBoxes>(this->boundingBoxesSubscription, 1));

  this->boundingBoxesCache->registerCallback(std::bind(&ImageYoloFilter::timer_callback, this));

  // this->sync.reset(new message_filters::Synchronizer<approximate_policy>(
  //     approximate_policy(10), alignedPictureSubscription, alignedPointcloudSubscription, boundingBoxesSubscription));
  // this->sync->registerCallback(std::bind(&ImageYoloFilter::syncMessageSubscription, this, std::placeholders::_1,
  //                                        std::placeholders::_2, std::placeholders::_3));
  // this->_sync.reset(
  //     new Sync(MySyncPolicy(10), alignedPictureSubscription, alignedPointcloudSubscription,
  //     boundingBoxesSubscription));
  // this->_sync->registerCallback(std::bind(&ImageYoloFilter::syncMessageSubscription, this, _1, _2, _3));

  this->lastDataTimeStamp = this->get_clock()->now();

  this->debug = this->get_parameter("debug").get_parameter_value().get<bool>();
  this->timeLagTolerance = this->get_parameter("timeLagToTolerance").get_parameter_value().get<float>();
  this->lagInNanoSecond = std::pow(10, 9) * (this->timeLagTolerance);
}

void ImageYoloFilter::timer_callback()
{

  if (this->debug)
  {
    this->calculationTimeStamp = this->get_clock()->now();
  }

  const rclcpp::Time latestTimeStamp = this->boundingBoxesCache->getLatestTime();

  // uint32_t lag = 100000000;
  // RCLCPP_INFO(this->get_logger() ,"second: %" PRIu32 " nanosec: %" PRIu32 " difference %" PRIu32
  // , latestTimeStamp.seconds(), latestTimeStamp.nanoseconds(), latestTimeStamp.nanoseconds() - lag);
  // rclcpp::Duration::from_nanoseconds(100000000)

  // RCLCPP_INFO(this->get_logger(), "lag:second %d in second %f", this->lagInNanoSecond, this->timeLagTolerance);
  const rclcpp::Time res = latestTimeStamp - rclcpp::Duration::from_nanoseconds(this->lagInNanoSecond);

  // double temp = latestTimeStamp.seconds() + latestTimeStamp.nanoseconds()/ std::pow(10,9);
  // double timeFinal =  temp -   (this->timeLagTolerance/std::pow(10,9) ) ;
  // uint64_t calculateNanoSec = timeFinal * std::pow(10,9);

  // RCLCPP_INFO(this->get_logger(), "time total is %lf, final is %lf, in unsigned is %" PRIu64 " ",temp, timeFinal,
  // calculateNanoSec );

  // RCLCPP_INFO(this->get_logger(), "1:second %lf  nano%ld", latestTimeStamp.seconds(), latestTimeStamp.nanoseconds());
  // RCLCPP_INFO(this->get_logger(), "2:second %lf  nano%ld", res.seconds(), res.nanoseconds());
  // this->objectBoundingBoxes = this->boundingBoxesCache->getElemBeforeTime(latestTimeStamp);
  // this->alignedImage = this->imageCache->getElemBeforeTime(latestTimeStamp);
  // this->alignedPointcloud = this->pointcloudCache->getElemBeforeTime(latestTimeStamp);

  auto boxes = this->boundingBoxesCache->getInterval(res, latestTimeStamp);
  if (boxes.size() > 0)
  {
    this->objectBoundingBoxes = boxes.back();
  }

  auto images = this->imageCache->getInterval(res, latestTimeStamp);
  if (images.size() > 0)
  {
    this->alignedImage = images.back();
  }

  auto pointclouds = this->pointcloudCache->getInterval(res, latestTimeStamp);
  if (pointclouds.size() > 0)
  {
    this->alignedPointcloud = pointclouds.back();
  }
  // this->alignedImage = this->imageCache->getInterval(res,latestTimeStamp);
  // this->alignedPointcloud = this->pointcloudCache->getInterval(res,latestTimeStamp);

  if (isMessageWithinTimeTolerance(this->timeLagTolerance))
  {
    // clear all remaining  data from last call

    this->filteredPointCloud.header = this->alignedPointcloud->header;

    // height
    // width

    this->filteredPointCloud.fields = this->alignedPointcloud->fields;
    this->filteredPointCloud.is_bigendian = this->alignedPointcloud->is_bigendian;
    this->filteredPointCloud.point_step = this->alignedPointcloud->point_step;
    // row_step
    // data
    // this->filteredPointCloud.is_dense = this->alignedPointcloud->is_dense;
    // processPointCloud();
    // this->filteredPointCloud.is_dense=false;

    // for debug purpose

    if (this->debug)
    {
      auto timeDifference = this->get_clock()->now() - this->lastDataTimeStamp;
      RCLCPP_INFO(this->get_logger(), "time lagged for collecting data is %f",
                  timeDifference.seconds() + timeDifference.nanoseconds() / std::pow(10, 9));
    }

    if (this->debug)
    {
      this->filteredPointCloud.data.clear();

      processPointCloudInDens();
      // debugging purpose
      this->filteredPointCloud.is_dense = false;
      // pcl::toROSMsg(*(this->alignedPointcloud.get()), filteredImage);
      pcl::toROSMsg((this->filteredPointCloud), filteredImage);
      filteredImage.header.frame_id = "camera_color_optical_frame";

      filteredImage.header = this->filteredPointCloud.header;
      this->imagePublisher->publish(this->filteredImage);
      // // convertToPng(filteredImage, "/home/shouyu/ros2_ws/filered.png");

      // // pcl::toROSMsg((*(this->alignedPointcloud.get())), filteredImage);
      // // convertToPng(filteredImage, "/home/shouyu/ros2_ws/unfilered.png");
    }
    else
    {
      processPointCloud();
    }

    this->pointcloudPublisher->publish(this->filteredPointCloud);
    this->lastDataTimeStamp = this->get_clock()->now();

    if (this->debug)
    {
      auto timeDifference = this->get_clock()->now() - this->calculationTimeStamp;
      RCLCPP_INFO(this->get_logger(), "time lagged for filtering data is %f",
                  timeDifference.seconds() + timeDifference.nanoseconds() / std::pow(10, 9));
    }
  }
}

void ImageYoloFilter::syncMessageSubscription(const sensor_msgs::msg::Image::ConstSharedPtr alignedImage,
                                              const sensor_msgs::msg::PointCloud2::ConstSharedPtr alignedPointCloud,
                                              const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr boundingBoxes)
{
  this->alignedImage = alignedImage;
  this->alignedPointcloud = alignedPointCloud;
  this->objectBoundingBoxes = boundingBoxes;
  this->timer_callback();  // process the newest
}

bool ImageYoloFilter::isMessageWithinTimeTolerance(double toleranceSecond)
{

  if (this->alignedPointcloud == nullptr || this->alignedImage == nullptr || this->objectBoundingBoxes == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "waiting for data to be initialized");
    return false;
  }
  else
  {
    double timeOfAlignedImage =
        this->alignedImage->header.stamp.sec + (this->alignedImage->header.stamp.nanosec) / (std::pow(10, 9));
    double timeOfBoundingBox = this->objectBoundingBoxes->header.stamp.sec +
                               (this->objectBoundingBoxes->header.stamp.nanosec) / (std::pow(10, 9));
    double timeOfAlignedPointcloud =
        this->alignedPointcloud->header.stamp.sec + (this->alignedPointcloud->header.stamp.nanosec) / (std::pow(10, 9));

    bool tolerance = std::abs(timeOfAlignedImage - timeOfBoundingBox) < toleranceSecond &&
                     std::abs(timeOfAlignedImage - timeOfAlignedPointcloud) < toleranceSecond &&
                     std::abs(timeOfAlignedPointcloud - timeOfBoundingBox) < toleranceSecond;

    if (!tolerance)
    {
      RCLCPP_WARN(this->get_logger(), "Messaged lagged out of tolerance value: time difference %f %f %f",
                  timeOfAlignedImage, timeOfBoundingBox, timeOfAlignedPointcloud);
    }
    return tolerance;
  }
}

void ImageYoloFilter::processPointCloudInDens()
{
  this->filteredPointCloud.is_dense = false;  // since there are invalid data
  // output a dense version of the point cloud
  int numberOfPoint = 0;
  int_fast64_t k, i, startIndex, endIndex;
  k = i = startIndex = endIndex = -1;
  const bboxes_ex_msgs::msg::BoundingBox* box_info;
  uint16_t image_height = this->alignedImage->height;
  uint16_t image_width = this->alignedImage->width;

  int debugCount = 0;
  // this->filteredPointCloud.data =
  // std::vector<std::numeric_limits<float>>(19,std::numeric_limits<float>::quiet_NaN());
  for (uint16_t ii = 0; ii < this->alignedImage->height; ii++)
  {
    for (uint16_t kk = 0; kk < this->alignedImage->width; kk++)
    {
      for (uint16_t q = 0; q < this->alignedPointcloud->point_step; q++)
      {
        this->filteredPointCloud.data.push_back(std::numeric_limits<uint8_t>::quiet_NaN());
        debugCount++;
      }
    }
  }

  try
  {
    for (size_t boxNum = 0; boxNum < this->objectBoundingBoxes->bounding_boxes.size(); boxNum++)
    {
      box_info = &this->objectBoundingBoxes->bounding_boxes.at(boxNum);

      for (i = std::max(0, box_info->ymin - 1); i < std::min(box_info->ymax, image_height); i++)
      {
        for (k = std::max(0, box_info->xmin - 1); k < std::min(box_info->xmax, image_width); k++)
        {
          startIndex =
              i * this->alignedPointcloud->point_step * alignedPointcloud->width + k * alignedPointcloud->point_step;
          endIndex = startIndex + alignedPointcloud->point_step;

          // int pointByteSize = alignedPointcloud->point_step / alignedPointcloud->fields.size();
          for (int dataInd = startIndex; dataInd < endIndex; dataInd++)
          {
            this->filteredPointCloud.data[dataInd] = (this->alignedPointcloud->data.at(dataInd));
          }

          numberOfPoint++;
        }
      }
    }

    this->filteredPointCloud.height = this->alignedImage->height;
    this->filteredPointCloud.width = this->alignedImage->width;
    this->filteredPointCloud.row_step = this->filteredPointCloud.point_step * this->alignedImage->width;
  }
  catch (std::out_of_range& e)
  {
    std::cout << "i : " << i << std::endl;
    std::cout << "k: " << k << std::endl;
    std::cout << " start Index" << startIndex << " end index" << endIndex << std::endl;
    std::cout << "xmin ,xmax" << box_info->xmin << "," << box_info->xmax << std::endl;
    std::cout << "ymin ,ymax" << box_info->ymin << "," << box_info->ymax << std::endl;
    std::cout << "img_height, img_width" << image_height << "," << image_width << std::endl;
  }
}
void ImageYoloFilter::processPointCloud()
{
  this->filteredPointCloud.is_dense = this->alignedPointcloud->is_dense;
  int numberOfPoint = 0;
  int k, i, startIndex, endIndex;
  k = i = startIndex = endIndex = -1;
  uint16_t image_height = this->alignedImage->height;
  uint16_t image_width = this->alignedImage->width;
  const bboxes_ex_msgs::msg::BoundingBox* box_info;

  size_t datapoint = 0;
  auto vectorSize = this->filteredPointCloud.data.size();
  try
  {
    for (size_t boxNum = 0; boxNum < this->objectBoundingBoxes->bounding_boxes.size(); boxNum++)
    {
      box_info = &this->objectBoundingBoxes->bounding_boxes.at(boxNum);

      for (i = std::max(0, box_info->ymin - 1); i < std::min(box_info->ymax, image_height); i++)
      {
        for (k = std::max(0, box_info->xmin - 1); k < std::min(box_info->xmax, image_width); k++)
        {
          startIndex =
              i * this->alignedPointcloud->point_step * alignedPointcloud->width + k * alignedPointcloud->point_step;
          endIndex = startIndex + alignedPointcloud->point_step;

          // int pointByteSize = alignedPointcloud->point_step / alignedPointcloud->fields.size();
          for (int dataInd = startIndex; dataInd < endIndex; dataInd++)
          {
            if (datapoint >= vectorSize)
            {
              this->filteredPointCloud.data.push_back(this->alignedPointcloud->data.at(dataInd));
            }
            else
            {
              this->filteredPointCloud.data.at(datapoint) = this->alignedPointcloud->data.at(dataInd);
            }
            datapoint++;  // for every info in the vector
            this->filteredPointCloud.data.push_back(this->alignedPointcloud->data.at(dataInd));
          }

          numberOfPoint++;
        }
      }
    }

    long long sizeDiff = this->filteredPointCloud.data.size() - datapoint;
    if (sizeDiff > 0)
    {
      this->filteredPointCloud.data.resize(datapoint);

    }
    this->filteredPointCloud.height = 1;
    this->filteredPointCloud.width = numberOfPoint;
    this->filteredPointCloud.row_step = this->filteredPointCloud.point_step * numberOfPoint;
  }
  catch (std::out_of_range& e)
  {
    std::cout << "i : " << i << std::endl;
    std::cout << "k: " << k << std::endl;
    std::cout << " start Index" << startIndex << " end index" << endIndex << std::endl;
    std::cout << "xmin ,xmax" << box_info->xmin << "," << box_info->xmax << std::endl;
    std::cout << "ymin ,ymax" << box_info->ymin << "," << box_info->ymax << std::endl;
    std::cout << "img_height, img_width" << image_height << "," << image_width << std::endl;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageYoloFilter>());
  rclcpp::shutdown();
  return 0;
}

