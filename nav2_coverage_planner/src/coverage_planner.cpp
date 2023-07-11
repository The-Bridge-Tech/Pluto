
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include <cmath>
#include "nav2_coverage_planner/coverage_planner.hpp"
#include "nav2_coverage_planner/coverage.hpp"
#include "nav2_coverage_planner/costmap_2d_modified.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <memory>
#include "coverage_area_interface/srv/select_square.hpp"

namespace nav2_coverage_planner
{

  void CoveragePlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    transform_tolerance_ = costmap_ros->getTransformTolerance();
    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".xy_goal_tolerance", rclcpp::ParameterValue( // get specific parameter for this node
                                                 0.3));
    // node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    this->interpolation_resolution_ = costmap_->getResolution();
    node_->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance);

    // coverageMap = nav2_costmap_2d::Costmap2D(134,118, 0.05,0,0,0);
    plannnerMapPub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(parent, &coverageMap, "planner", "planner/map", true);

    // create local planner
    planner_ = std::make_unique<Coverage>(
        0,
        0);

    // make the coveragePose vector to be size 0
    this->coveragePose.clear();

    // TODO: make this as parameter?
    planner_frame_ = "planner";
    movedToOrigin = false;
    planner_path_index = 0;

    // TODO: initalize costmap here?
    this->needUpdateCoverageMap = true;
    this->costmap_last_x_cell = costmap_->getSizeInCellsX();
    this->costmap_last_y_cell = costmap_->getSizeInCellsY();
    // extractPlannerCostMap();

    coverageService = this->node_->create_service<coverage_area_interface::srv::SelectSquare>("SelectSquare", std::bind(&CoveragePlanner::receiveNewCoverageArea, this, std::placeholders::_1, std::placeholders::_2));
  }

  void CoveragePlanner::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type CoveragePlanner",
        name_.c_str());
    plannnerMapPub->on_cleanup();
  }

  void CoveragePlanner::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type CoveragePlanner",
        name_.c_str());

    plannnerMapPub->on_activate();

    // try to extract and transform the costmap that we want to our current frame
  }

  void CoveragePlanner::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type CoveragePlanner",
        name_.c_str());
    plannnerMapPub->on_deactivate();
  }
  void CoveragePlanner::extractPlannerCostMap()
  {
    RCLCPP_INFO(node_->get_logger(), "x, y %d %d", this->costmap_->getSizeInCellsX(), this->costmap_->getSizeInCellsY());
    // std::vector<double> pose = {-2, 0, 2, 0,
    //                             -2, -2, 2, -2};
    // x, y of each coordidnates
    // std::vector<double> pose = {0, 0, 1, -1,
    //                             -1, -1, 0, -2};
    auto pose = coveragePose;
    if (pose.size() != 8)
    {
      RCLCPP_ERROR(node_->get_logger(), "Incorrect coveragePose Size");
    }

    std::vector<nav2_costmap_2d::MapLocation> map_polygon;
    for (size_t i = 0; i < pose.size(); i += 2)
    {
      nav2_costmap_2d::MapLocation loc;
      if (!this->costmap_->worldToMap(pose[i], pose[i + 1], loc.x, loc.y))
      {
        // failed to convert;
        RCLCPP_INFO(node_->get_logger(), "extractPlannerCostMap(): failed to convert x,y: %f %f", pose[i], pose[i + 1]);
      }
      else
      {
        RCLCPP_INFO(logger_, "extractPlannerCostMap(): i %d j %d", loc.x, loc.y);
      }

      map_polygon.push_back(loc);
    }

    std::vector<nav2_costmap_2d::MapLocation> coverageLocation;

    // nav2_costmap_2d::Costmap2D resMap(134,118,this->costmap_->getResolution(), 0,0);//= this->costmap_->extractPolygonMap(map_polygon);
    unsigned int x_size;
    unsigned int y_size;
    double map_res;
    // nav2_costmap_2d::Costmap2D debugMap = this->costmap_->extractPolygonMap(map_polygon, coverageLocation, x_size, y_size,false);

    //
    // this->costmap_->extractPolygonMap(map_polygon, coverageLocation, x_size, y_size, false);

    nav2_costmap_2d::Costmap2DModified extractTool(
        this->costmap_->getSizeInCellsX(),
        this->costmap_->getSizeInCellsY(),
        this->costmap_->getResolution(),
        this->costmap_->getOriginX(),
        this->costmap_->getOriginY(),
        0

    );
    // nav2_costmap_2d::Costmap2DModified extractTool;

    extractTool.extractCostmap(map_polygon[0], map_polygon[1], map_polygon[2], map_polygon[3], coverageLocation, x_size, y_size);

    // extractTool.extractCostmap(
    //     tempCost,
    //     map_polygon[0], map_polygon[1], map_polygon[2], map_polygon[3],
    //     coverageLocation, x_size, y_size);
    // extractTool.extractCostmap(map_polygon, coverageLocation, x_size, y_size);
    //   create a new map
    map_res = costmap_->getResolution();

    double x_size_in_meter = std::sqrt(std::pow((pose[0] - pose[2]), 2) + std::pow((pose[1] - pose[3]), 2));
    double y_size_in_meter = std::sqrt(std::pow((pose[0] - pose[4]), 2) + std::pow((pose[1] - pose[5]), 2));

    // TOOD: need to round up?

    // TODO: very important note: robot should never touch the 4 corner,
    //  moreover, should never stay or cross the boundary map
    x_size = std::ceil(x_size_in_meter / map_res); // TODO: is this correct?
    y_size = std::ceil(y_size_in_meter / map_res);

    coverageMap = nav2_costmap_2d::Costmap2D(x_size, y_size, map_res, (-1 * (x_size_in_meter) / 2), (-1 * (y_size_in_meter) / 2));
    // coverageMap = nav2_costmap_2d::Costmap2D(x_size, y_size, map_res, 0.0, 0.0);
    // coverageMap.resizeMap(x_size, y_size, map_res, (-1 * (x_size * map_res) / 2), (-1 * (y_size * map_res) / 2) );
    RCLCPP_INFO(this->node_->get_logger(), "x_size %d y_size %d x_origin %f y_origin %f resolution %f", coverageMap.getSizeInCellsX(),
                coverageMap.getSizeInCellsY(), coverageMap.getOriginX(), coverageMap.getOriginY(), coverageMap.getResolution());

    // debug action

    // for(std::size_t i = 0; i <coverageLocation.size(); i++){
    //   // convert the i,j to meters
    //   double x_world, y_world;
    //   double x_world2, y_world2;
    //   this->costmap_->mapToWorld(coverageLocation[i].x, coverageLocation[i].y, x_world2, y_world2);
    //   mapToWorld(coverageLocation[i].x, coverageLocation[i].y, x_world, y_world, this->costmap_);
    //   RCLCPP_INFO(this->node_->get_logger(), "index: %d %d meter: %f %f  2nd %f %f", coverageLocation[i].x, coverageLocation[i].y, x_world, y_world,x_world2, y_world2);
    // }
    for (std::size_t i = 0; i < coverageLocation.size(); i++)
    {
      geometry_msgs::msg::PoseStamped coveragePose;
      geometry_msgs::msg::PoseStamped mapPose;

      // 1. transform the index to meter unit for "map" frame

      double map_x, map_y;
      costmap_->mapToWorld(coverageLocation[i].x, coverageLocation[i].y, map_x, map_y);
      mapPose.pose.position.x = map_x;
      mapPose.pose.position.y = map_y;
      mapPose.pose.position.z = 0.0;
      mapPose.pose.orientation.x = 0.0;
      mapPose.pose.orientation.y = 0.0;
      mapPose.pose.orientation.z = 0.0;
      mapPose.pose.orientation.w = 1.0; // don't really care about the orientation right now.
      mapPose.header.stamp = node_->now();
      mapPose.header.frame_id = global_frame_;
      transformPoseToAnotherFrame(mapPose, coveragePose, planner_frame_); // transform to planner pose;

      auto cost = this->costmap_->getCost(coverageLocation[i].x, coverageLocation[i].y);

      // 2. get the  map index of this "planner" map base on the tf result
      unsigned int planner_m_x, planner_m_y;

      if (!coverageMap.worldToMap(coveragePose.pose.position.x, coveragePose.pose.position.y, planner_m_x, planner_m_y))
      {
        RCLCPP_INFO(node_->get_logger(), "Failed to convert from world to map!! map : %f %f to  planner %f %f", map_x, map_y, coveragePose.pose.position.x, coveragePose.pose.position.y);
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

    // debugMap.saveMap("debugMap.pgm");
    // coverageMap.saveMap("Testmap.pgm");
    // RCLCPP_INFO(
    //     node_->get_logger(), "save map!!!");

    auto xSize = coverageMap.getSizeInMetersX();
    auto ySize = coverageMap.getSizeInMetersY();
    auto resolution = coverageMap.getResolution();

    RCLCPP_INFO(node_->get_logger(), "x meter %f ymeter %f res %f", xSize, ySize, resolution);

    // try to publish the extracted map
    plannnerMapPub->publishCostmap();
  }

  nav_msgs::msg::Path CoveragePlanner::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;
    auto curr_start = start;
    auto cur_goal = goal;

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    if (this->coveragePose.size() != 8)
    {
      RCLCPP_INFO(logger_, "Waiting for servive on /SelectArea");
      return global_path;
    }
    // // clear the starting cell within the costmap because we know it can't be an obstacle
    // //TODO transform that  into "map" frame first?
    // clearRobotCell(mx, my);

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // make sure to resize the underlying array that Coverage uses
    // Update planner based on the new costmap size

    if (isPlannerOutOfDate())
    {
      needUpdateCoverageMap = false;
      movedToOrigin = false; // let robot move back to origin
      costmap_last_x_cell = this->costmap_->getSizeInCellsX();
      costmap_last_y_cell = this->costmap_->getSizeInCellsY();

      extractPlannerCostMap();
      planner_->setNavArr(
          this->coverageMap.getSizeInCellsX(),
          this->coverageMap.getSizeInCellsY());
      planner_->setupPathArray(coverageMap.getSizeInCellsX() * coverageMap.getSizeInCellsY());

      RCLCPP_INFO(logger_, "detect change of map size, did you reset the global costmap?");
      planner_->setCostmap(coverageMap.getCharMap(), true);
    }

    lock.unlock();

    // Checking if the goal and start state is in the global frame
    if (curr_start.header.frame_id != planner_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner try to transform position to %s frame",
          planner_frame_.c_str());
      transformPoseToAnotherFrame(start, curr_start, planner_frame_);
      // return global_path;
    }

    if (cur_goal.header.frame_id != planner_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner try to transform position to %s frame",
          planner_frame_.c_str());
      transformPoseToAnotherFrame(goal, cur_goal, planner_frame_);
      // return global_path;
    }

    double wx = curr_start.pose.position.x;
    double wy = curr_start.pose.position.y;

    // RCLCPP_DEBUG(
    //     logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    //     start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    unsigned int mx, my;
    if (!this->coverageMap.worldToMap(wx, wy, mx, my))
    {
      RCLCPP_WARN(
          logger_,
          "Cannot create a plan: the robot's start position is off the global"
          " costmap. Planning will always fail, are you sure"
          " the robot has been properly localized?");
    }
    else
    {
      // RCLCPP_INFO(logger_, "start: transform from [meter] %f %f to [matrix] %u %u", wx, wy, mx, my);
    }

    int start_index[2];
    start_index[0] = mx;
    start_index[1] = my;

    int planner_start[2]; // start row, col index of planner to start planning
    planner_start[0] = 0; // mx;
    planner_start[1] = 0;

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
      planner_origin_pose.header.stamp = node_->now();
      planner_origin_pose.header.frame_id = planner_frame_;

      linearInterpolation(curr_start, planner_origin_pose, global_path);
      // linearInterpolation(curr_start, cur_goal, global_path);
      RCLCPP_INFO(logger_, "origin:  planner [meter] %f %f, index %d %d", planner_origin_x, planner_origin_y, planner_start[0], planner_start[1]);

      // see if their difference is within the xy_goal_tolerance
      double distance = std::hypot(curr_start.pose.position.x - planner_origin_pose.pose.position.x,
                                   curr_start.pose.position.y - planner_origin_pose.pose.position.y);
      distance = std::floor(distance * 100.0) / 100.0;
      RCLCPP_INFO(logger_, "remaining distance %f, tolerance distance %f", distance, xy_goal_tolerance);
      if (distance <= xy_goal_tolerance)
      {
        RCLCPP_INFO(logger_, "setting it to true");
        movedToOrigin = true;
      }
    }
    else
    {

      if (planner_->getPathLen() == 0)
      {

        //planner_->savemap("beforePlanner.pgm"); // debug purpose
        if (!planner_->setStart(planner_start))
        {
          RCLCPP_ERROR(logger_, "unexpected error when initialize planner with start");
          // since is not on one of the edge

          // use assume to move to the edge
        }

        planner_->squareMovement();

        //planner_->savemap("afterPlanner.pgm"); debug purpose
      }
      else
      {
        for (int j = 0; j < this->planner_->getPathLen(); j++)
        {

          mapXY p = this->planner_->getPathXY()[j];

          if (start_index[0] == p.x && start_index[1] == p.y)
          {
            planner_path_index = j;
            RCLCPP_INFO(logger_, "The index for path is %d", planner_path_index); //debug purpose
          }
        }

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
          planner_pose.header.stamp = node_->now();
          planner_pose.header.frame_id = planner_frame_;
          latestPost = planner_pose;
          transformPoseToAnotherFrame(planner_pose, map_pose, global_frame_);

          // RCLCPP_INFO(logger_, "coordinates x: %d y: %d   map x: %f y %f planner x: %f y: %f", p.x, p.y, map_pose.pose.position.x, map_pose.pose.position.y,worldX,worldY);
          global_path.poses.push_back(map_pose);
        }
        // RCLCPP_INFO(logger_, "The origin of planner costmap x: %f y: %f", coverageMap.getOriginX(), coverageMap.getOriginY());

        // linearInterpolation(curr_start, latestPost, global_path);
      }
    }

    // at last, do something that move to the goal_pose

    // wx = cur_goal.pose.position.x;
    // wy = cur_goal.pose.position.y;
    // //TODO: any relvant in this case?
    // if (! this->coverageMap.worldToMap(wx, wy, mx, my))
    // {
    //   RCLCPP_WARN(
    //       logger_,
    //       "The goal sent to the planner is off the global costmap."
    //       " Planning will always fail to this goal.");
    // }

    // RCLCPP_INFO(logger_, "goal: transform from [meter] %f %f to [matrix] %u %u",wx,wy, mx, my );

    // int map_goal[2];
    // map_goal[0] = mx;
    // map_goal[1] = my;

    // TODO: later

    RCLCPP_INFO(
        node_->get_logger(), "Planner has total %ld pose in path", global_path.poses.size());

    return global_path;
  }

  void CoveragePlanner::linearInterpolation(const geometry_msgs::msg::PoseStamped start, const geometry_msgs::msg::PoseStamped goal, nav_msgs::msg::Path &global_path)
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
      planner_pose.header.stamp = node_->now();
      planner_pose.header.frame_id = global_frame_;
      global_path.poses.push_back(planner_pose);
    }

    geometry_msgs::msg::PoseStamped goal_pose = cur_goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
  }

  bool CoveragePlanner::transformPoseToAnotherFrame(const geometry_msgs::msg::PoseStamped &input_pose,
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
  bool CoveragePlanner::transformPoseToGlobalFrame(const geometry_msgs::msg::PoseStamped &input_pose,
                                                   geometry_msgs::msg::PoseStamped &transformed_pose)
  {
    if (input_pose.header.frame_id == global_frame_)
    {
      transformed_pose = input_pose;
      return true;
    }
    else
    {
      return nav2_util::transformPoseInTargetFrame(
          input_pose, transformed_pose, *(this->tf_),
          global_frame_, transform_tolerance_);
    }
  }

  // bool CoveragePlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const

  //   double origin_x_ = costmap_->getOriginX();
  //   double origin_y_ = costmap->getOriginY();
  //   if (wx < costmap_->get_ || wy < origin_y_)
  //   {
  //     RCLCPP_ERROR(
  //       node_->get_logger(), "smaller than origin %d %d origin %d %", wx, wy, );
  //     return false;
  //   }

  //   mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  //   my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  //   double size_x_ = costmap_->get
  //   if (mx < size_x_ && my < size_y_)
  //   {

  //     return true;
  //   }
  //   RCLCPP_ERROR(
  //       node_->get_logger(), "index bigger than size %d %d  maxIndex %d %d", mx,my,
  //     )
  //   return false;
  // }

  // bool
  // CoveragePlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my, nav2_costmap_2d::Costmap2D *costmap)
  // {
  //   if (wx < costmap->getOriginX() || wy < costmap->getOriginY())
  //   {
  //     return false;
  //   }

  //   mx = static_cast<int>(
  //       std::round((wx - costmap->getOriginX()) / costmap->getResolution()));
  //   my = static_cast<int>(
  //       std::round((wy - costmap->getOriginY()) / costmap->getResolution()));

  //   if (mx < costmap->getSizeInCellsX() && my < costmap->getSizeInCellsY())
  //   {
  //     return true;
  //   }

  //   RCLCPP_ERROR(
  //       logger_,
  //       "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
  //       coverageMap.getSizeInCellsX(), coverageMap.getSizeInCellsY());

  //   return false;
  // }

  // void
  // CoveragePlanner::mapToWorld(double mx, double my, double &wx, double &wy, nav2_costmap_2d::Costmap2D *costmap)
  // {
  //   wx = costmap->getOriginX() + mx * costmap->getResolution();
  //   wy = costmap->getOriginY() + my * costmap->getResolution();
  // }
  void CoveragePlanner::clearRobotCell(unsigned int mx, unsigned int my)
  {

    costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  }

  // TODO: update the logic in here
  bool CoveragePlanner::isPlannerOutOfDate()
  {

    if (!planner_.get() ||
        costmap_last_x_cell != static_cast<int>(costmap_->getSizeInCellsX()) ||
        costmap_last_y_cell != static_cast<int>(costmap_->getSizeInCellsY()) ||
        needUpdateCoverageMap)
    {
      return true;
    }
    return false;
  }

  // TODO: test for multiple times replace the coverage area?
  void CoveragePlanner::receiveNewCoverageArea(const std::shared_ptr<coverage_area_interface::srv::SelectSquare::Request> request, // CHANGE
                                               std::shared_ptr<coverage_area_interface::srv::SelectSquare::Response> response)
  {

    // this->coveragePose is store in the following sequence
    // lower_left x,y
    // upper_left x,y
    // lower_right x,y
    // upper_right x,y
    this->coveragePose.clear();
    this->coveragePose.push_back(request->lower_left_x);
    this->coveragePose.push_back(request->lower_left_y);
    this->coveragePose.push_back(request->upper_left_x);
    this->coveragePose.push_back(request->upper_left_y);
    this->coveragePose.push_back(request->lower_right_x);
    this->coveragePose.push_back(request->lower_right_y);
    this->coveragePose.push_back(request->upper_right_x);
    this->coveragePose.push_back(request->upper_right_y);

    this->needUpdateCoverageMap = true;

    RCLCPP_INFO(
        logger_, "lower_left_x: %f lower_left_y: %f upper_left_x: %f upper_left_y: %f lower_right_x: %f lower_right_y: %f upper_right_x: %f upper_right_y: %f ",
        request->lower_left_x, request->lower_left_y,
        request->upper_left_x, request->upper_left_y,
        request->lower_right_x, request->lower_right_y,
        request->upper_right_x, request->upper_right_y

    );
    response->result = true;
  }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_coverage_planner::CoveragePlanner, nav2_core::GlobalPlanner)
