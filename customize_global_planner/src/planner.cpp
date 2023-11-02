#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "customize_global_planner/coverage.hpp"
#include "customize_global_planner/costmap_2d_modified.hpp"
#include "customize_global_planner/planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
// /* This example creates a subclass of Node and uses std::bind() to register a
// * member function as a callback from the timer. */

namespace customize_global_planner
{

  GlobalPlanner::GlobalPlanner()
      : Node("global_planner"), count_(0)
  {

    this->declare_parameter("odometryTopic", "/odom"); // TODO: change to /odometry/global

    this->declare_parameter("xy_goal_tolerance", 0.5);
    this->declare_parameter("global_planner_frequency", 10);

    std::string odometryTopic = this->get_parameter("odometryTopic").get_parameter_value().get<std::string>();
    this->xy_goal_tolerance = this->get_parameter("xy_goal_tolerance").get_parameter_value().get<double>();

    double planner_frequency = this->get_parameter("global_planner_frequency").get_parameter_value().get<int>();


    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    unsigned int milliSecond = (1/planner_frequency)*1000;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds( (milliSecond) ), std::bind(&GlobalPlanner::timer_callback, this));

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&GlobalPlanner::map_subscriber_callback, this, std::placeholders::_1));
    global_odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometryTopic, 10, std::bind(&GlobalPlanner::odometry_subscriber_callback, this, std::placeholders::_1));
    planner_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("planner/map", 10);
    planner_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planner/path", 10);
    coverageService = this->create_service<coverage_area_interface::srv::SelectSquare>("SelectSquare",

                                                                                       std::bind(&GlobalPlanner::receiveNewCoverageArea, this, std::placeholders::_1, std::placeholders::_2));

    tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    name_ = "customizePlanner";
    global_frame_ = "map"; // TODO: change later

    // nav2_util::LifecycleNode node2("testNode0");
    // node_ = std::make_unique<nav2_util::LifecycleNode>("testNode0");
    // plannnerMapPub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(node_, &coverageMap, "planner", "planner/map", true);

    // create planner_ for coveragePathPlanning
    planner_ = std::make_unique<Coverage>(
        0,
        0);

    this->coveragePose.clear(); // make the coveragePose vector to be size 0,

    // TODO: make this as parameter?
    planner_frame_ = "planner"; // The frame id of the "coverage planner"
    movedToOrigin = false;
    planner_path_index = 0;

    this->needUpdateCoverageMap = true; // indicate that need to extract costmap for coverage algorithm

    planner_start[0] = 0;
    planner_start[1] = 0;

    coverageService = this->create_service<coverage_area_interface::srv::SelectSquare>("SelectSquare", std::bind(&GlobalPlanner::receiveNewCoverageArea, this, std::placeholders::_1, std::placeholders::_2));

    // while(1){

    //   RCLCPP_INFO(this->get_logger(),"spin");
    // }
  }

  void GlobalPlanner::timer_callback()
  {
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);

    geometry_msgs::msg::PoseStamped currentLocation;
    currentLocation.header = this->globalOdometry.header;
    currentLocation.pose = this->globalOdometry.pose.pose;
    if (this->coveragePose.size() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for coverage area info");
    }
    else
    {
      createPlan(currentLocation);
    }
  }

  void GlobalPlanner::publishPlannerMap()
  {

    nav_msgs::msg::OccupancyGrid occupancy_grid;

    // Copy header information from the Costmap2D
    std_msgs::msg::Header mapHeader;
    mapHeader.stamp = this->now();
    mapHeader.frame_id = "planner";
    occupancy_grid.header = mapHeader;
    occupancy_grid.info.map_load_time = this->now();
    // Set the width and height of the occupancy grid
    occupancy_grid.info.width = coverageMap.getSizeInCellsX();
    occupancy_grid.info.height = coverageMap.getSizeInCellsY();

    // Set the resolution
    occupancy_grid.info.resolution = coverageMap.getResolution();
    geometry_msgs::msg::Pose originPoint;
    originPoint.position.x = coverageMap.getOriginX();
    originPoint.position.y = coverageMap.getOriginY();
    occupancy_grid.info.origin = originPoint;
    // Convert the costmap data to the occupancy grid data

    // occupancy_grid.data = std::vector<int8_t>(coverageMap.getCharMap(), (coverageMap.getCharMap() + coverageMap.getSizeInCellsX()*coverageMap.getSizeInCellsY()));
    occupancy_grid.data = std::vector<int8_t>();

    for (unsigned long i = 0; i < coverageMap.getSizeInCellsX() * coverageMap.getSizeInCellsY(); i++)
    {
      occupancy_grid.data.push_back(coverageMap.getCharMap()[i]);
    }
    // // plannnerMapPub->publishCostmap();
    planner_map_publisher_->publish(occupancy_grid);
    RCLCPP_INFO(this->get_logger(), "Publish map");
  }

  void GlobalPlanner::odometry_subscriber_callback(const nav_msgs::msg::Odometry &odometryData)
  {
    this->globalOdometry = odometryData;
  }
  void GlobalPlanner::map_subscriber_callback(const nav_msgs::msg::OccupancyGrid &mapMessage)
  {

    RCLCPP_INFO(this->get_logger(), "Received costmap data");

    costMap = nav2_costmap_2d::Costmap2D(mapMessage);
    if (this->costmap_ == nullptr)
    {
      // means is first time receive map
      costmap_ = &costMap;
      this->interpolation_resolution_ = costMap.getResolution();
    }
    else
    {
      costmap_ = &costMap;
    }

    // record the global costmap's x,y size. Use for determine change in global costmap, like reloading a costmap.
    this->costmap_last_x_cell = costmap_->getSizeInCellsX();
    this->costmap_last_y_cell = costmap_->getSizeInCellsY();

    // plannnerMapPub->on_activate();

    // pure testing
    // coverageMap = costMap;
    // this->publishPlannerMap();
    // nav_msgs::msg::OccupancyGrid occupancy_grid;

    // // Copy header information from the Costmap2D
    // std_msgs::msg::Header mapHeader;
    // mapHeader.stamp = this->now();
    // mapHeader.frame_id = "planner";
    // occupancy_grid.header = mapHeader;
    // occupancy_grid.info.map_load_time = this->now();
    // // Set the width and height of the occupancy grid
    // occupancy_grid.info.width = coverageMap.getSizeInCellsX();
    // occupancy_grid.info.height = coverageMap.getSizeInCellsY();

    // // Set the resolution
    // occupancy_grid.info.resolution = coverageMap.getResolution();
    // geometry_msgs::msg::Pose originPoint;
    // originPoint.position.x = coverageMap.getOriginX();
    // originPoint.position.y = coverageMap.getOriginY();
    // occupancy_grid.info.origin = originPoint;
    // // Convert the costmap data to the occupancy grid data

    // // occupancy_grid.data = std::vector<int8_t>(coverageMap.getCharMap(), (coverageMap.getCharMap() + coverageMap.getSizeInCellsX()*coverageMap.getSizeInCellsY()));
    // occupancy_grid.data = std::vector<int8_t>();

    // for (unsigned long i = 0; i < coverageMap.getSizeInCellsX() * coverageMap.getSizeInCellsY(); i++)
    // {
    //   occupancy_grid.data.push_back(coverageMap.getCharMap()[i]);
    // }
    // // // plannnerMapPub->publishCostmap();
    // planner_map_publisher_->publish(occupancy_grid);
    // RCLCPP_INFO(this->get_logger(), "Publish map");
  }

  void GlobalPlanner::extractPlannerCostMap()
  {
    RCLCPP_INFO(this->get_logger(), "costmapSize:  x, y %d %d", this->costmap_->getSizeInCellsX(), this->costmap_->getSizeInCellsY());

    if (coveragePose.size() != 8) // check if has 4 x,y (the corner of the rectangle area to cover)
    {
      RCLCPP_ERROR(this->get_logger(), "Incorrect coveragePose Size");
    }

    // now, turn the 4 (x,y) point into costmap_2d:MapLocation. This is a preparation for extracting coverage costmap from the global costmap
    std::vector<nav2_costmap_2d::MapLocation> map_polygon;
    for (size_t i = 0; i < coveragePose.size(); i += 2) // turn those four coordinate into nav2_costmap_2d::MapLocation object
    {
      nav2_costmap_2d::MapLocation loc;
      if (!this->costmap_->worldToMap(coveragePose[i], coveragePose[i + 1], loc.x, loc.y))
      {
        // failed to convert;
        RCLCPP_INFO(this->get_logger(), "extractPlannerCostMap(): failed to convert x,y: %f %f", coveragePose[i], coveragePose[i + 1]);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "extractPlannerCostMap(): i %d j %d", loc.x, loc.y);
      }

      map_polygon.push_back(loc);
    }

    std::vector<nav2_costmap_2d::MapLocation> coverageLocation; // Vector use to store all (x,y) grid location (in global costmap) that are within the coverage costmap (define by the 4 x,y)
    unsigned int x_size;
    unsigned int y_size;
    double map_res;

    nav2_costmap_2d::Costmap2DModified extractTool(
        this->costmap_->getSizeInCellsX(),
        this->costmap_->getSizeInCellsY(),
        this->costmap_->getResolution(),
        this->costmap_->getOriginX(),
        this->costmap_->getOriginY(),
        0

    );

    // call function to extractCostmap
    extractTool.extractCostmap(map_polygon[0], map_polygon[1], map_polygon[2], map_polygon[3], coverageLocation, x_size, y_size); // extract all grid base on the 4 corner defined

    //   create a new map
    map_res = costmap_->getResolution();
    // using pythagorean theorem
    double x_size_in_meter = std::sqrt(std::pow((coveragePose[0] - coveragePose[2]), 2) + std::pow((coveragePose[1] - coveragePose[3]), 2));
    double y_size_in_meter = std::sqrt(std::pow((coveragePose[0] - coveragePose[4]), 2) + std::pow((coveragePose[1] - coveragePose[5]), 2));

    x_size = std::ceil(x_size_in_meter / map_res);
    y_size = std::ceil(y_size_in_meter / map_res);

    coverageMap = nav2_costmap_2d::Costmap2D(x_size, y_size, map_res, (-1 * (x_size_in_meter) / 2), (-1 * (y_size_in_meter) / 2));
    RCLCPP_INFO(this->get_logger(), "x_size %d y_size %d x_origin %f y_origin %f resolution %f", coverageMap.getSizeInCellsX(),
                coverageMap.getSizeInCellsY(), coverageMap.getOriginX(), coverageMap.getOriginY(), coverageMap.getResolution());

    for (std::size_t i = 0; i < coverageLocation.size(); i++)
    {
      geometry_msgs::msg::PoseStamped coveragePose;
      geometry_msgs::msg::PoseStamped mapPose;

      // 1. transform the index (in 2d grid array) to meter unit for "map" frame, and then use TF tree to transform pose from "map" to planner_frame_
      double map_x, map_y;
      costmap_->mapToWorld(coverageLocation[i].x, coverageLocation[i].y, map_x, map_y);
      // mapToWorld(coverageLocation[i].x, coverageLocation[i].y, map_x, map_y, costmap_);
      mapPose.pose.position.x = map_x;
      mapPose.pose.position.y = map_y;
      mapPose.pose.position.z = 0.0;
      mapPose.pose.orientation.x = 0.0;
      mapPose.pose.orientation.y = 0.0;
      mapPose.pose.orientation.z = 0.0;
      mapPose.pose.orientation.w = 1.0; // don't really care about the orientation right now.
      mapPose.header.stamp = this->now();
      mapPose.header.frame_id = global_frame_;
      transformPoseToAnotherFrame(mapPose, coveragePose, planner_frame_); // transform to planner frame pose;

      auto cost = this->costmap_->getCost(coverageLocation[i].x, coverageLocation[i].y);

      // 2. get the  map index(in 2d grid array) of this planner_frame_
      unsigned int planner_m_x, planner_m_y;

      if (!coverageMap.worldToMap(coveragePose.pose.position.x, coveragePose.pose.position.y, planner_m_x, planner_m_y))
      {
        RCLCPP_INFO(this->get_logger(), "Failed to convert from world to map!!  costmap x,y %u %u map : %f %f to  planner %f %f", coverageLocation[i].x, coverageLocation[i].y, map_x, map_y, coveragePose.pose.position.x, coveragePose.pose.position.y);
      }
      else
      {
        // 3. set the cost to coverageMap
        coverageMap.setCost(planner_m_x, planner_m_y, cost);
      }

      // TODO: turn on to debug the minor bug
      //  RCLCPP_INFO(node_->get_logger(), "from x %f y %f index %d %d   to  x %f y %f index %d %d",
      //              map_x, map_y, coverageLocation[i].x, coverageLocation[i].y, coveragePose.pose.position.x, coveragePose.pose.position.y,
      //              planner_m_x, planner_m_y);
    }

    // coverageMap.saveMap("Testmap.pgm");
    // RCLCPP_INFO(
    //     node_->get_logger(), "save map!!!");

    auto xSize = coverageMap.getSizeInMetersX();
    auto ySize = coverageMap.getSizeInMetersY();
    auto resolution = coverageMap.getResolution();

    RCLCPP_INFO(this->get_logger(), "x meter %f ymeter %f res %f", xSize, ySize, resolution);

    // try to publish the coverage costmap that is extracted from global costmap
    // plannnerMapPub->publishCostmap();
    this->publishPlannerMap();
  }

  nav_msgs::msg::Path GlobalPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped &start)
  {

    nav_msgs::msg::Path global_path;
    auto curr_start = start;

    global_path.poses.clear();
    global_path.header.stamp = this->now();
    global_path.header.frame_id = global_frame_;

    if (this->coveragePose.size() != 8)
    {
      RCLCPP_INFO(logger_, "Waiting for service on /SelectArea");
      return global_path;
    }

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // make sure to resize the underlying array that Coverage uses
    // Update planner based on the new costmap size

    if (isPlannerOutOfDate())
    {
      planNewPath();
    }

    lock.unlock();

    // Checking if the goal and start state is in the global frame
    if (curr_start.header.frame_id != planner_frame_)
    {
      RCLCPP_INFO(
          this->get_logger(), "Planner try to transform position to %s frame",
          planner_frame_.c_str());
      transformPoseToAnotherFrame(start, curr_start, planner_frame_);
    }

    // RCLCPP_DEBUG(
    //     logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    //     start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    double wx = curr_start.pose.position.x;
    double wy = curr_start.pose.position.y;
    unsigned int mx, my;
    if (!this->coverageMap.worldToMap(wx, wy, mx, my))
    {
      RCLCPP_WARN(
          logger_,
          "Cannot create a plan: the robot's start position is off the global"
          " costmap. Planning will always fail, are you sure"
          " the robot has been properly localized?");
    }

    int start_pose_x_y[2];
    start_pose_x_y[0] = mx;
    start_pose_x_y[1] = my;

    if (movedToOrigin == false)
    {

      // // 1. use cur_star as the start position
      double planner_origin_x, planner_origin_y;
      geometry_msgs::msg::PoseStamped planner_origin_pose;

      // make robot to 0,0 (note: in the middle of the grid)
      coverageMap.mapToWorld(planner_start[0], planner_start[1], planner_origin_x, planner_origin_y);

      planner_origin_pose.pose.position.x = planner_origin_x;
      planner_origin_pose.pose.position.y = planner_origin_y;
      planner_origin_pose.pose.position.z = 0.0;
      planner_origin_pose.pose.orientation.x = 0.0;
      planner_origin_pose.pose.orientation.y = 0.0;
      planner_origin_pose.pose.orientation.z = 0.0;
      planner_origin_pose.pose.orientation.w = 1.0;
      planner_origin_pose.header.stamp = this->now();
      planner_origin_pose.header.frame_id = planner_frame_;
      linearInterpolation(curr_start, planner_origin_pose, global_path); // generate a straight path from curr_start(current position of the robot in "planner_frame_") to the (0,0 in  coverageMap)[already translated to "planner_frame_"]
      RCLCPP_INFO(this->get_logger(), "origin:  planner [meter] %f %f, index %d %d", planner_origin_x, planner_origin_y, planner_start[0], planner_start[1]);

      // see if their difference is within the xy_goal_tolerance
      double distance = std::hypot(curr_start.pose.position.x - planner_origin_pose.pose.position.x,
                                   curr_start.pose.position.y - planner_origin_pose.pose.position.y);
      distance = std::floor(distance * 100.0) / 100.0;
      RCLCPP_INFO(this->get_logger(), "remaining distance %f, tolerance distance %f", distance, xy_goal_tolerance);
      if (distance <= xy_goal_tolerance)
      {
        RCLCPP_INFO(this->get_logger(), "setting it to true");
        movedToOrigin = true; // when the distance is within the tolerance.
      }
    }
    else
    {

      // second stage of planner(after moved to (0,0) grid in coverageMap)
      if (planner_->getPathLen() == 0)
      {
        RCLCPP_FATAL(this->get_logger(), "No Coverage path, unexpected error!");
      }
      else
      {
        // means already have a calculated path. Now add the poses in the path to global_path
        for (int j = 0; j < this->planner_->getPathLen(); j++)
        {

          mapXY p = this->planner_->getPathXY()[j];

          if (start_pose_x_y[0] == p.x && start_pose_x_y[1] == p.y)
          {
            planner_path_index = j;
            RCLCPP_INFO(this->get_logger(), "The index for path is %d in total of %d ", planner_path_index, this->planner_->getPathLen()); // debug purpose
          }
        }

        if (planner_path_index + 1 == this->planner_->getPathLen())
        {
          // means already at the end
          RCLCPP_INFO(this->get_logger(), "At the end of planner path!");
        }
        else
        {
          global_path.poses.push_back(start); // in map coordinates
          geometry_msgs::msg::PoseStamped latestPost;
          for (int i = planner_path_index; i < this->planner_->getPathLen(); i++)
          {
            geometry_msgs::msg::PoseStamped planner_pose;
            geometry_msgs::msg::PoseStamped map_pose;

            double worldX = 0;
            double worldY = 0;
            mapXY p = this->planner_->getPathXY()[i];

            coverageMap.mapToWorld(p.x, p.y, worldX, worldY);

            planner_pose.pose.position.x = worldX;
            planner_pose.pose.position.y = worldY;
            planner_pose.pose.position.z = 0.0;
            planner_pose.pose.orientation.x = 0.0;
            planner_pose.pose.orientation.y = 0.0;
            planner_pose.pose.orientation.z = 0.0;
            planner_pose.pose.orientation.w = 1.0;
            planner_pose.header.stamp = this->now();
            planner_pose.header.frame_id = planner_frame_;
            latestPost = planner_pose;
            transformPoseToAnotherFrame(planner_pose, map_pose, global_frame_);

            // RCLCPP_INFO(logger_, "coordinates x: %d y: %d   map x: %f y %f planner x: %f y: %f", p.x, p.y, map_pose.pose.position.x, map_pose.pose.position.y,worldX,worldY);
            global_path.poses.push_back(map_pose);
          }
        }

        // debug purpose
        // RCLCPP_INFO(
        //     node_->get_logger(), "The last pose of the planner x: %f y: %f", global_path.poses.back().pose.position.x, global_path.poses.back().pose.position.y);
      }
    }

    // Optional: at last, do something that move to the goal_pose

    RCLCPP_INFO(
        this->get_logger(), "Planner has total %ld pose in path", global_path.poses.size());

    planner_path_publisher_->publish(global_path);
    return global_path;
  }

  void GlobalPlanner::linearInterpolation(const geometry_msgs::msg::PoseStamped start, const geometry_msgs::msg::PoseStamped goal, nav_msgs::msg::Path &global_path)
  {

    // calculating the number of loops for current value of interpolation_resolution_
    geometry_msgs::msg::PoseStamped cur_goal;
    geometry_msgs::msg::PoseStamped curr_start;

    // first, convert both curr_start and cur_goal into the _global_frame
    transformPoseToAnotherFrame(start, curr_start, global_frame_);
    transformPoseToAnotherFrame(goal, cur_goal, global_frame_);

    // RCLCPP_INFO(logger_, "start:  map from [meter] %f %f", curr_start.pose.position.x, curr_start.pose.position.y);
    // RCLCPP_INFO(logger_, "origin:  map from [meter] %f %f", cur_goal.pose.position.x, cur_goal.pose.position.x);
    //  second, do the linear extrapolation
    int total_number_of_loop = std::hypot(
                                   cur_goal.pose.position.x - curr_start.pose.position.x,
                                   cur_goal.pose.position.y - curr_start.pose.position.y) /
                               interpolation_resolution_;
    double x_increment = (cur_goal.pose.position.x - curr_start.pose.position.x) / total_number_of_loop;
    double y_increment = (cur_goal.pose.position.y - curr_start.pose.position.y) / total_number_of_loop;

    for (int i = 0; i < total_number_of_loop; ++i)
    {
      geometry_msgs::msg::PoseStamped planner_pose;
      planner_pose.pose.position.x = curr_start.pose.position.x + x_increment * i;
      planner_pose.pose.position.y = curr_start.pose.position.y + y_increment * i;
      planner_pose.pose.position.z = 0.0;
      planner_pose.pose.orientation.x = 0.0;
      planner_pose.pose.orientation.y = 0.0;
      planner_pose.pose.orientation.z = 0.0;
      planner_pose.pose.orientation.w = 1.0;
      planner_pose.header.stamp = this->now();
      planner_pose.header.frame_id = global_frame_;
      global_path.poses.push_back(planner_pose);
    }

    geometry_msgs::msg::PoseStamped goal_pose = cur_goal;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
  }
  bool GlobalPlanner::transformPoseToAnotherFrame(const geometry_msgs::msg::PoseStamped &input_pose,
                                                  geometry_msgs::msg::PoseStamped &transformed_pose, const std::string &goalFrame)
  {

    if (input_pose.header.frame_id == goalFrame)
    {
      transformed_pose = input_pose;
      return true;
    }
    else
    {
      return nav2_util::transformPoseInTargetFrame(
          input_pose, transformed_pose, *(this->tf_),
          goalFrame, transform_tolerance_);
    }
  }
  bool GlobalPlanner::isPlannerOutOfDate()
  {

    if (
        costmap_last_x_cell != static_cast<unsigned int>(costmap_->getSizeInCellsX()) ||
        costmap_last_y_cell != static_cast<unsigned int>(costmap_->getSizeInCellsY()) ||
        needUpdateCoverageMap)
    {
      return true;
    }
    return false;
  }
  void GlobalPlanner::receiveNewCoverageArea(const std::shared_ptr<coverage_area_interface::srv::SelectSquare::Request> request, // CHANGE
                                             std::shared_ptr<coverage_area_interface::srv::SelectSquare::Response> response)
  {

    // first store the area of cutting
    this->coveragePose.clear();
    this->coveragePose.push_back(request->lower_left_x);
    this->coveragePose.push_back(request->lower_left_y);
    this->coveragePose.push_back(request->upper_left_x);
    this->coveragePose.push_back(request->upper_left_y);
    this->coveragePose.push_back(request->lower_right_x);
    this->coveragePose.push_back(request->lower_right_y);
    this->coveragePose.push_back(request->upper_right_x);
    this->coveragePose.push_back(request->upper_right_y);

    this->needUpdateCoverageMap = true; // every time when receiving a new coverage area, need to call update the coverage Map and redo the planning

    RCLCPP_INFO(
        logger_, "lower_left_x: %f lower_left_y: %f upper_left_x: %f upper_left_y: %f lower_right_x: %f lower_right_y: %f upper_right_x: %f upper_right_y: %f ",
        request->lower_left_x, request->lower_left_y,
        request->upper_left_x, request->upper_left_y,
        request->lower_right_x, request->lower_right_y,
        request->upper_right_x, request->upper_right_y

    );

    planNewPath();
    // convert the planner_origin
    double planner_end_x, planner_end_y;
    geometry_msgs::msg::PoseStamped final_end_pose, map_pose;

    mapXY p = this->planner_->getPathXY()[this->planner_->getPathLen() - 1];

    coverageMap.mapToWorld(p.x, p.y, planner_end_x, planner_end_y);

    final_end_pose.pose.position.x = planner_end_x;
    final_end_pose.pose.position.y = planner_end_y;
    final_end_pose.pose.position.z = 0.0;
    final_end_pose.pose.orientation.x = 0.0;
    final_end_pose.pose.orientation.y = 0.0;
    final_end_pose.pose.orientation.z = 0.0;
    final_end_pose.pose.orientation.w = 1.0;
    final_end_pose.header.stamp = this->now();
    final_end_pose.header.frame_id = planner_frame_;

    transformPoseToAnotherFrame(final_end_pose, map_pose, global_frame_);

    response->goal_end_pose = map_pose;
  }

  void GlobalPlanner::planNewPath()
  {
    needUpdateCoverageMap = false; // indicate start a new round of updating the planner

    movedToOrigin = false; // let robot move back to origin
    planner_start[0] = 0;  // the origin -x of the "planner" frame grid
    planner_start[1] = 0;

    costmap_last_x_cell = this->costmap_->getSizeInCellsX();
    costmap_last_y_cell = this->costmap_->getSizeInCellsY();

    extractPlannerCostMap();
    planner_->setNavArr(
        this->coverageMap.getSizeInCellsX(),
        this->coverageMap.getSizeInCellsY());
    planner_->setupPathArray(coverageMap.getSizeInCellsX() * coverageMap.getSizeInCellsY());

    RCLCPP_INFO(logger_, "detect change of map size, did you reset the global costmap?");
    planner_->setCostmap(coverageMap.getCharMap(), true);

    // planner_->savemap("beforePlanner.pgm"); // debug purpose
    if (!planner_->setStart(planner_start))
    {
      RCLCPP_ERROR(logger_, "unexpected error when initialize planner with start");
      // since is not on one of the edge
      // use assume to move to the edge
    }

    planner_->squareMovement(); // calculate a coverage path

    // planner_->savemap("afterPlanner.pgm"); //debug purpose
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<customize_global_planner::GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}

// #include <cmath>
// #include <string>
// #include <memory>
// #include "nav2_util/node_utils.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "nav2_costmap_2d/costmap_2d.hpp"
// #include "nav2_costmap_2d/costmap_2d_publisher.hpp"
// #include <cmath>
// #include "customize_global_planner/planner.hpp"
// #include "customize_global_planner/coverage.hpp"
// #include "customize_global_planner/costmap_2d_modified.hpp"
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <memory>
// #include "coverage_area_interface/srv/select_square.hpp"

// namespace nav2_coverage_planner
// {

//   void CoveragePlanner::CoveragePlanner():nav2_util::LifecycleNode("coverage_planner","",true){

//   }
//   void CoveragePlanner::configure(
//       const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
//       std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
//   {
//     node_ = parent.lock();
//     name_ = name;
//     tf_ = tf;
//     costmap_ = costmap_ros->getCostmap();
//     global_frame_ = costmap_ros->getGlobalFrameID();
//     transform_tolerance_ = costmap_ros->getTransformTolerance();
//     // Parameter initialization
//     nav2_util::declare_parameter_if_not_declared(
//         node_, name_ + ".xy_goal_tolerance", rclcpp::ParameterValue( // get specific parameter for this node
//                                                  0.3));
//     // node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
//     this->interpolation_resolution_ = costmap_->getResolution();
//     node_->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance);

//     // coverageMap = nav2_costmap_2d::Costmap2D(134,118, 0.05,0,0,0);  debug purpose
//     plannnerMapPub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(parent, &coverageMap, "planner", "planner/map", true);

//     // create planner_ for coveragePathPlanning
//     planner_ = std::make_unique<Coverage>(
//         0,
//         0);

//     this->coveragePose.clear(); // make the coveragePose vector to be size 0,

//     // TODO: make this as parameter?
//     planner_frame_ = "planner"; // The frame id of the "coverage planner"
//     movedToOrigin = false;
//     planner_path_index = 0;

//     this->needUpdateCoverageMap = true;  // indicate that need to extract costmap for coverage algorithm

//     // record the global costmap's x,y size. Use for determine change in global costmap, like reloading a costmap.
//     this->costmap_last_x_cell = costmap_->getSizeInCellsX();
//     this->costmap_last_y_cell = costmap_->getSizeInCellsY();

//     planner_start[0] = 0;
//     planner_start[1] = 0;

//     coverageService = this->node_->create_service<coverage_area_interface::srv::SelectSquare>("SelectSquare", std::bind(&CoveragePlanner::receiveNewCoverageArea, this, std::placeholders::_1, std::placeholders::_2));
//   }

//   void CoveragePlanner::cleanup()
//   {
//     RCLCPP_INFO(
//         node_->get_logger(), "CleaningUp plugin %s of type CoveragePlanner",
//         name_.c_str());
//     plannnerMapPub->on_cleanup();
//   }

//   void CoveragePlanner::activate()
//   {
//     RCLCPP_INFO(
//         node_->get_logger(), "Activating plugin %s of type CoveragePlanner",
//         name_.c_str());

//     plannnerMapPub->on_activate();

//     // try to extract and transform the costmap that we want to our current frame
//   }

//   void CoveragePlanner::deactivate()
//   {
//     RCLCPP_INFO(
//         node_->get_logger(), "Deactivating plugin %s of type CoveragePlanner",
//         name_.c_str());
//     plannnerMapPub->on_deactivate();
//   }
//   void CoveragePlanner::extractPlannerCostMap()
//   {
//     RCLCPP_INFO(node_->get_logger(), "costmapSize:  x, y %d %d", this->costmap_->getSizeInCellsX(), this->costmap_->getSizeInCellsY());

//     if (coveragePose.size() != 8) // check if has 4 x,y (the corner of the rectangle area to cover)
//     {
//       RCLCPP_ERROR(node_->get_logger(), "Incorrect coveragePose Size");
//     }

//     // now, turn the 4 (x,y) point into costmap_2d:MapLocation. This is a preparation for extracting coverage costmap from the global costmap
//     std::vector<nav2_costmap_2d::MapLocation> map_polygon;
//     for (size_t i = 0; i < coveragePose.size(); i += 2) // turn those four coordinate into nav2_costmap_2d::MapLocation object
//     {
//       nav2_costmap_2d::MapLocation loc;
//       if (!this->costmap_->worldToMap(coveragePose[i], coveragePose[i + 1], loc.x, loc.y))
//       {
//         // failed to convert;
//         RCLCPP_INFO(node_->get_logger(), "extractPlannerCostMap(): failed to convert x,y: %f %f", coveragePose[i], coveragePose[i + 1]);
//       }
//       else
//       {
//         RCLCPP_INFO(logger_, "extractPlannerCostMap(): i %d j %d", loc.x, loc.y);
//       }

//       map_polygon.push_back(loc);
//     }

//     std::vector<nav2_costmap_2d::MapLocation> coverageLocation; // Vector use to store all (x,y) grid location (in global costmap) that are within the coverage costmap (define by the 4 x,y)
//     unsigned int x_size;
//     unsigned int y_size;
//     double map_res;

//     nav2_costmap_2d::Costmap2DModified extractTool(
//         this->costmap_->getSizeInCellsX(),
//         this->costmap_->getSizeInCellsY(),
//         this->costmap_->getResolution(),
//         this->costmap_->getOriginX(),
//         this->costmap_->getOriginY(),
//         0

//     );

//     // call function to extractCostmap
//     extractTool.extractCostmap(map_polygon[0], map_polygon[1], map_polygon[2], map_polygon[3], coverageLocation, x_size, y_size); // extract all grid base on the 4 corner defined

//     //   create a new map
//     map_res = costmap_->getResolution();
//     // using pythagorean theorem
//     double x_size_in_meter = std::sqrt(std::pow((coveragePose[0] - coveragePose[2]), 2) + std::pow((coveragePose[1] - coveragePose[3]), 2));
//     double y_size_in_meter = std::sqrt(std::pow((coveragePose[0] - coveragePose[4]), 2) + std::pow((coveragePose[1] - coveragePose[5]), 2));

//     x_size = std::ceil(x_size_in_meter / map_res);
//     y_size = std::ceil(y_size_in_meter / map_res);

//     coverageMap = nav2_costmap_2d::Costmap2D(x_size, y_size, map_res, (-1 * (x_size_in_meter) / 2), (-1 * (y_size_in_meter) / 2));
//     RCLCPP_INFO(this->node_->get_logger(), "x_size %d y_size %d x_origin %f y_origin %f resolution %f", coverageMap.getSizeInCellsX(),
//                 coverageMap.getSizeInCellsY(), coverageMap.getOriginX(), coverageMap.getOriginY(), coverageMap.getResolution());

//     for (std::size_t i = 0; i < coverageLocation.size(); i++)
//     {
//       geometry_msgs::msg::PoseStamped coveragePose;
//       geometry_msgs::msg::PoseStamped mapPose;

//       // 1. transform the index (in 2d grid array) to meter unit for "map" frame, and then use TF tree to transform pose from "map" to planner_frame_
//       double map_x, map_y;
//       costmap_->mapToWorld(coverageLocation[i].x, coverageLocation[i].y, map_x, map_y);
//       // mapToWorld(coverageLocation[i].x, coverageLocation[i].y, map_x, map_y, costmap_);
//       mapPose.pose.position.x = map_x;
//       mapPose.pose.position.y = map_y;
//       mapPose.pose.position.z = 0.0;
//       mapPose.pose.orientation.x = 0.0;
//       mapPose.pose.orientation.y = 0.0;
//       mapPose.pose.orientation.z = 0.0;
//       mapPose.pose.orientation.w = 1.0; // don't really care about the orientation right now.
//       mapPose.header.stamp = node_->now();
//       mapPose.header.frame_id = global_frame_;
//       transformPoseToAnotherFrame(mapPose, coveragePose, planner_frame_); // transform to planner frame pose;

//       auto cost = this->costmap_->getCost(coverageLocation[i].x, coverageLocation[i].y);

//       // 2. get the  map index(in 2d grid array) of this planner_frame_
//       unsigned int planner_m_x, planner_m_y;

//       if (!coverageMap.worldToMap(coveragePose.pose.position.x, coveragePose.pose.position.y, planner_m_x, planner_m_y))
//       {
//         RCLCPP_INFO(node_->get_logger(), "Failed to convert from world to map!!  costmap x,y %u %u map : %f %f to  planner %f %f", coverageLocation[i].x, coverageLocation[i].y, map_x, map_y, coveragePose.pose.position.x, coveragePose.pose.position.y);
//       }
//       else
//       {
//         // 3. set the cost to coverageMap
//         coverageMap.setCost(planner_m_x, planner_m_y, cost);
//       }

//       // TODO: turn on to debug the minor bug
//       //  RCLCPP_INFO(node_->get_logger(), "from x %f y %f index %d %d   to  x %f y %f index %d %d",
//       //              map_x, map_y, coverageLocation[i].x, coverageLocation[i].y, coveragePose.pose.position.x, coveragePose.pose.position.y,
//       //              planner_m_x, planner_m_y);
//     }

//     // coverageMap.saveMap("Testmap.pgm");
//     // RCLCPP_INFO(
//     //     node_->get_logger(), "save map!!!");

//     auto xSize = coverageMap.getSizeInMetersX();
//     auto ySize = coverageMap.getSizeInMetersY();
//     auto resolution = coverageMap.getResolution();

//     RCLCPP_INFO(node_->get_logger(), "x meter %f ymeter %f res %f", xSize, ySize, resolution);

//     // try to publish the coverage costmap that is extracted from global costmap
//     plannnerMapPub->publishCostmap();
//   }

//   nav_msgs::msg::Path CoveragePlanner::createPlan(
//       const geometry_msgs::msg::PoseStamped &start,
//       const geometry_msgs::msg::PoseStamped &goal)
//   {
//     nav_msgs::msg::Path global_path;
//     auto curr_start = start;
//     auto cur_goal = goal;

//     global_path.poses.clear();
//     global_path.header.stamp = node_->now();
//     global_path.header.frame_id = global_frame_;

//     if (this->coveragePose.size() != 8)
//     {
//       RCLCPP_INFO(logger_, "Waiting for service on /SelectArea");
//       return global_path;
//     }

//     std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

//     // make sure to resize the underlying array that Coverage uses
//     // Update planner based on the new costmap size

//     if (isPlannerOutOfDate())
//     {
//         planNewPath();
//     }

//     lock.unlock();

//     // Checking if the goal and start state is in the global frame
//     if (curr_start.header.frame_id != planner_frame_)
//     {
//       RCLCPP_INFO(
//           node_->get_logger(), "Planner try to transform position to %s frame",
//           planner_frame_.c_str());
//       transformPoseToAnotherFrame(start, curr_start, planner_frame_);
//     }

//     if (cur_goal.header.frame_id != planner_frame_)
//     {
//       RCLCPP_INFO(
//           node_->get_logger(), "Planner try to transform cur_goal to %s frame %f %f",
//           planner_frame_.c_str(), cur_goal.pose.position.x, cur_goal.pose.position.y);
//       transformPoseToAnotherFrame(goal, cur_goal, planner_frame_);
//     }

//     // RCLCPP_DEBUG(
//     //     logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
//     //     start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
//     double wx = curr_start.pose.position.x;
//     double wy = curr_start.pose.position.y;
//     unsigned int mx, my;
//     if (!this->coverageMap.worldToMap(wx, wy, mx, my))
//     {
//       RCLCPP_WARN(
//           logger_,
//           "Cannot create a plan: the robot's start position is off the global"
//           " costmap. Planning will always fail, are you sure"
//           " the robot has been properly localized?");
//     }

//     int start_pose_x_y[2];
//     start_pose_x_y[0] = mx;
//     start_pose_x_y[1] = my;

//     if (movedToOrigin == false)
//     {

//       // // 1. use cur_star as the start position
//       double planner_origin_x, planner_origin_y;
//       geometry_msgs::msg::PoseStamped planner_origin_pose;

//       // make robot to 0,0 (note: in the middle of the grid)
//       coverageMap.mapToWorld(planner_start[0], planner_start[1], planner_origin_x, planner_origin_y);

//       planner_origin_pose.pose.position.x = planner_origin_x;
//       planner_origin_pose.pose.position.y = planner_origin_y;
//       planner_origin_pose.pose.position.z = 0.0;
//       planner_origin_pose.pose.orientation.x = 0.0;
//       planner_origin_pose.pose.orientation.y = 0.0;
//       planner_origin_pose.pose.orientation.z = 0.0;
//       planner_origin_pose.pose.orientation.w = 1.0;
//       planner_origin_pose.header.stamp = node_->now();
//       planner_origin_pose.header.frame_id = planner_frame_;

//       linearInterpolation(curr_start, planner_origin_pose, global_path); // generate a straight path from curr_start(current position of the robot in "planner_frame_") to the (0,0 in  coverageMap)[already translated to "planner_frame_"]
//       // linearInterpolation(curr_start, cur_goal, global_path);
//       RCLCPP_INFO(logger_, "origin:  planner [meter] %f %f, index %d %d", planner_origin_x, planner_origin_y, planner_start[0], planner_start[1]);

//       // see if their difference is within the xy_goal_tolerance
//       double distance = std::hypot(curr_start.pose.position.x - planner_origin_pose.pose.position.x,
//                                    curr_start.pose.position.y - planner_origin_pose.pose.position.y);
//       distance = std::floor(distance * 100.0) / 100.0;
//       RCLCPP_INFO(logger_, "remaining distance %f, tolerance distance %f", distance, xy_goal_tolerance);
//       if (distance <= xy_goal_tolerance)
//       {
//         RCLCPP_INFO(logger_, "setting it to true");
//         movedToOrigin = true; // when the distance is within the tolerance.
//       }
//     }
//     else
//     {

//       // second stage of planner(after moved to (0,0) grid in coverageMap)
//       if (planner_->getPathLen() == 0)
//       {
//         RCLCPP_FATAL(logger_, "No Coverage path, unexpected error!");
//       }
//       else
//       {
//         // means already have a calculated path. Now add the poses in the path to global_path
//         for (int j = 0; j < this->planner_->getPathLen(); j++)
//         {

//           mapXY p = this->planner_->getPathXY()[j];

//           if (start_pose_x_y[0] == p.x && start_pose_x_y[1] == p.y)
//           {
//             planner_path_index = j;
//             RCLCPP_INFO(logger_, "The index for path is %d", planner_path_index); // debug purpose
//           }
//         }

//         global_path.poses.push_back(start); // in map coordinates
//         geometry_msgs::msg::PoseStamped latestPost;
//         for (int i = planner_path_index; i < this->planner_->getPathLen(); i++)
//         {
//           geometry_msgs::msg::PoseStamped planner_pose;
//           geometry_msgs::msg::PoseStamped map_pose;

//           double worldX = 0;
//           double worldY = 0;
//           mapXY p = this->planner_->getPathXY()[i];

//           coverageMap.mapToWorld(p.x, p.y, worldX, worldY);

//           planner_pose.pose.position.x = worldX;
//           planner_pose.pose.position.y = worldY;
//           planner_pose.pose.position.z = 0.0;
//           planner_pose.pose.orientation.x = 0.0;
//           planner_pose.pose.orientation.y = 0.0;
//           planner_pose.pose.orientation.z = 0.0;
//           planner_pose.pose.orientation.w = 1.0;
//           planner_pose.header.stamp = node_->now();
//           planner_pose.header.frame_id = planner_frame_;
//           latestPost = planner_pose;
//           transformPoseToAnotherFrame(planner_pose, map_pose, global_frame_);

//           // RCLCPP_INFO(logger_, "coordinates x: %d y: %d   map x: %f y %f planner x: %f y: %f", p.x, p.y, map_pose.pose.position.x, map_pose.pose.position.y,worldX,worldY);
//           global_path.poses.push_back(map_pose);
//         }

//         // debug purpose
//         // RCLCPP_INFO(
//         //     node_->get_logger(), "The last pose of the planner x: %f y: %f", global_path.poses.back().pose.position.x, global_path.poses.back().pose.position.y);

//       }
//     }

//     // Optional: at last, do something that move to the goal_pose

//     RCLCPP_INFO(
//         node_->get_logger(), "Planner has total %ld pose in path", global_path.poses.size());

//     return global_path;
//   }

//   void CoveragePlanner::planNewPath()
//   {
//     needUpdateCoverageMap = false; // indicate start a new round of updating the planner

//     movedToOrigin = false; // let robot move back to origin
//     planner_start[0] = 0;  // the origin -x of the "planner" frame grid
//     planner_start[1] = 0;

//     costmap_last_x_cell = this->costmap_->getSizeInCellsX();
//     costmap_last_y_cell = this->costmap_->getSizeInCellsY();

//     extractPlannerCostMap();
//     planner_->setNavArr(
//         this->coverageMap.getSizeInCellsX(),
//         this->coverageMap.getSizeInCellsY());
//     planner_->setupPathArray(coverageMap.getSizeInCellsX() * coverageMap.getSizeInCellsY());

//     RCLCPP_INFO(logger_, "detect change of map size, did you reset the global costmap?");
//     planner_->setCostmap(coverageMap.getCharMap(), true);

//     // planner_->savemap("beforePlanner.pgm"); // debug purpose
//     if (!planner_->setStart(planner_start))
//     {
//       RCLCPP_ERROR(logger_, "unexpected error when initialize planner with start");
//       // since is not on one of the edge
//       // use assume to move to the edge
//     }

//     planner_->squareMovement(); // calculate a coverage path

//     // planner_->savemap("afterPlanner.pgm"); //debug purpose
//   }
//   void CoveragePlanner::linearInterpolation(const geometry_msgs::msg::PoseStamped start, const geometry_msgs::msg::PoseStamped goal, nav_msgs::msg::Path &global_path)
//   {

//     // calculating the number of loops for current value of interpolation_resolution_
//     geometry_msgs::msg::PoseStamped cur_goal;
//     geometry_msgs::msg::PoseStamped curr_start;

//     // first, convert both curr_start and cur_goal into the _global_frame
//     transformPoseToAnotherFrame(start, curr_start, global_frame_);
//     transformPoseToAnotherFrame(goal, cur_goal, global_frame_);

//     // RCLCPP_INFO(logger_, "start:  map from [meter] %f %f", curr_start.pose.position.x, curr_start.pose.position.y);
//     // RCLCPP_INFO(logger_, "origin:  map from [meter] %f %f", cur_goal.pose.position.x, cur_goal.pose.position.x);
//     //  second, do the linear extrapolation
//     int total_number_of_loop = std::hypot(
//                                    cur_goal.pose.position.x - curr_start.pose.position.x,
//                                    cur_goal.pose.position.y - curr_start.pose.position.y) /
//                                interpolation_resolution_;
//     double x_increment = (cur_goal.pose.position.x - curr_start.pose.position.x) / total_number_of_loop;
//     double y_increment = (cur_goal.pose.position.y - curr_start.pose.position.y) / total_number_of_loop;

//     for (int i = 0; i < total_number_of_loop; ++i)
//     {
//       geometry_msgs::msg::PoseStamped planner_pose;
//       planner_pose.pose.position.x = curr_start.pose.position.x + x_increment * i;
//       planner_pose.pose.position.y = curr_start.pose.position.y + y_increment * i;
//       planner_pose.pose.position.z = 0.0;
//       planner_pose.pose.orientation.x = 0.0;
//       planner_pose.pose.orientation.y = 0.0;
//       planner_pose.pose.orientation.z = 0.0;
//       planner_pose.pose.orientation.w = 1.0;
//       planner_pose.header.stamp = node_->now();
//       planner_pose.header.frame_id = global_frame_;
//       global_path.poses.push_back(planner_pose);
//     }

//     geometry_msgs::msg::PoseStamped goal_pose = cur_goal;
//     goal_pose.header.stamp = node_->now();
//     goal_pose.header.frame_id = global_frame_;
//     global_path.poses.push_back(goal_pose);
//   }

//   bool CoveragePlanner::transformPoseToAnotherFrame(const geometry_msgs::msg::PoseStamped &input_pose,
//                                                     geometry_msgs::msg::PoseStamped &transformed_pose, const std::string &goalFrame)
//   {

//     if (input_pose.header.frame_id == goalFrame)
//     {
//       transformed_pose = input_pose;
//       return true;
//     }
//     else
//     {
//       return nav2_util::transformPoseInTargetFrame(
//           input_pose, transformed_pose, *(this->tf_),
//           goalFrame, transform_tolerance_);
//     }
//   }

//   // bool
//   // CoveragePlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my, nav2_costmap_2d::Costmap2D *costmap)
//   // {
//   //   if (wx < costmap->getOriginX() || wy < costmap->getOriginY())
//   //   {
//   //     return false;
//   //   }

//   //   mx = static_cast<int>(
//   //       std::round((wx - costmap->getOriginX()) / costmap->getResolution()));
//   //   my = static_cast<int>(
//   //       std::round((wy - costmap->getOriginY()) / costmap->getResolution()));

//   //   if (mx < costmap->getSizeInCellsX() && my < costmap->getSizeInCellsY())
//   //   {
//   //     return true;
//   //   }

//   //   RCLCPP_ERROR(
//   //       logger_,
//   //       "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
//   //       coverageMap.getSizeInCellsX(), coverageMap.getSizeInCellsY());

//   //   return false;
//   // }

//   // void
//   // CoveragePlanner::mapToWorld(double mx, double my, double &wx, double &wy, nav2_costmap_2d::Costmap2D *costmap)
//   // {
//   //   wx = costmap->getOriginX() + mx * costmap->getResolution();
//   //   wy = costmap->getOriginY() + my * costmap->getResolution();
//   // }
//   void CoveragePlanner::clearRobotCell(unsigned int mx, unsigned int my)
//   {

//     costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
//   }

//   // TODO: update the logic in here
//   bool CoveragePlanner::isPlannerOutOfDate()
//   {

//     if (!planner_.get() ||
//         costmap_last_x_cell != static_cast<int>(costmap_->getSizeInCellsX()) ||
//         costmap_last_y_cell != static_cast<int>(costmap_->getSizeInCellsY()) ||
//         needUpdateCoverageMap)
//     {
//       return true;
//     }
//     return false;
//   }

//   // TODO: test for multiple times replace the coverage area?
//   void CoveragePlanner::receiveNewCoverageArea(const std::shared_ptr<coverage_area_interface::srv::SelectSquare::Request> request, // CHANGE
//                                                std::shared_ptr<coverage_area_interface::srv::SelectSquare::Response> response)
//   {

//     // this->coveragePose is store in the following sequence
//     // lower_left x,y
//     // upper_left x,y
//     // lower_right x,y
//     // upper_right x,y
//     this->coveragePose.clear();
//     this->coveragePose.push_back(request->lower_left_x);
//     this->coveragePose.push_back(request->lower_left_y);
//     this->coveragePose.push_back(request->upper_left_x);
//     this->coveragePose.push_back(request->upper_left_y);
//     this->coveragePose.push_back(request->lower_right_x);
//     this->coveragePose.push_back(request->lower_right_y);
//     this->coveragePose.push_back(request->upper_right_x);
//     this->coveragePose.push_back(request->upper_right_y);

//     this->needUpdateCoverageMap = true; // every time when receiving a new coverage area, need to call update the coverage Map and redo the planning

//     RCLCPP_INFO(
//         logger_, "lower_left_x: %f lower_left_y: %f upper_left_x: %f upper_left_y: %f lower_right_x: %f lower_right_y: %f upper_right_x: %f upper_right_y: %f ",
//         request->lower_left_x, request->lower_left_y,
//         request->upper_left_x, request->upper_left_y,
//         request->lower_right_x, request->lower_right_y,
//         request->upper_right_x, request->upper_right_y

//     );

//     planNewPath();
//     // convert the planner_origin
//     double planner_end_x, planner_end_y;
//     geometry_msgs::msg::PoseStamped final_end_pose, map_pose;

//     mapXY p = this->planner_->getPathXY()[this->planner_->getPathLen()-1];

//     coverageMap.mapToWorld(p.x, p.y, planner_end_x, planner_end_y);

//     final_end_pose.pose.position.x = planner_end_x;
//     final_end_pose.pose.position.y = planner_end_y;
//     final_end_pose.pose.position.z = 0.0;
//     final_end_pose.pose.orientation.x = 0.0;
//     final_end_pose.pose.orientation.y = 0.0;
//     final_end_pose.pose.orientation.z = 0.0;
//     final_end_pose.pose.orientation.w = 1.0;
//     final_end_pose.header.stamp = node_->now();
//     final_end_pose.header.frame_id = planner_frame_;

//     transformPoseToAnotherFrame(final_end_pose, map_pose, global_frame_);

//     response->goal_end_pose = map_pose;
//   }

// }

// // #include <chrono>
// // #include <functional>
// // #include <memory>
// // #include <string>

// // #include "rclcpp/rclcpp.hpp"
// // #include "std_msgs/msg/string.hpp"

// // using namespace std::chrono_literals;

// // /* This example creates a subclass of Node and uses std::bind() to register a
// // * member function as a callback from the timer. */

// // class MinimalPublisher : public rclcpp::Node
// // {
// //   public:
// //     MinimalPublisher()
// //     : Node("minimal_publisher"), count_(0)
// //     {
// //       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
// //       timer_ = this->create_wall_timer(
// //       500ms, std::bind(&MinimalPublisher::timer_callback, this));
// //     }

// //   private:
// //     void timer_callback()
// //     {
// //       auto message = std_msgs::msg::String();
// //       message.data = "Hello, world! " + std::to_string(count_++);
// //       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
// //       publisher_->publish(message);
// //     }
// //     rclcpp::TimerBase::SharedPtr timer_;
// //     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
// //     size_t count_;
// // };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }