#ifndef __PUBLISH_FILTER_POINTCLOUD__

#define __PUBLISH_FILTER_POINTCLOUD__

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
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
#include <message_filters/sync_policies/approximate_time.h>
using namespace std::chrono_literals;

using approximate_policy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2,
                                                    bboxes_ex_msgs::msg::BoundingBoxes>;
typedef message_filters::Synchronizer<approximate_policy> Synchronizer;

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <fstream>
using namespace std::chrono_literals;
// using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;

using std::placeholders::_1;
class ImageYoloFilter : public rclcpp::Node
{
public:
  ImageYoloFilter();
  bool debug=false;
  void convertToPng(const sensor_msgs::msg::Image& image_msg, const std::string& filename);

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
  size_t count_;
  int qosCount;
  sensor_msgs::msg::PointCloud2 filteredPointCloud = sensor_msgs::msg::PointCloud2();
  sensor_msgs::msg::Image filteredImage = sensor_msgs::msg::Image();

  // void alignedPictureCallback(sensor_msgs::msg::Image::SharedPtr image);
  message_filters::Subscriber<sensor_msgs::msg::Image> alignedPictureSubscription;
  std::shared_ptr<const sensor_msgs::msg::Image> alignedImage;

  // void boundingBoxCallback(bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr boxInfo);
  message_filters::Subscriber<bboxes_ex_msgs::msg::BoundingBoxes> boundingBoxesSubscription;
  std::shared_ptr<const bboxes_ex_msgs::msg::BoundingBoxes> objectBoundingBoxes;

  // void alignedPointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> alignedPointcloudSubscription;
  std::shared_ptr<const sensor_msgs::msg::PointCloud2> alignedPointcloud;
  void syncMessageSubscription(const sensor_msgs::msg::Image::ConstSharedPtr alignedImage,
                               const sensor_msgs::msg::PointCloud2::ConstSharedPtr alignedPointCloud,
                               const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr boundingBoxes);

  bool isMessageWithinTimeTolerance(double toleranceSecond);

  void processPointCloud();
  void processPointCloudInDens();
  std::unique_ptr<Synchronizer> sync;
};

#endif