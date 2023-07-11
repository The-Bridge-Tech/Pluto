
#ifndef NAV2_COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_
#define NAV2_COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "nav2_coverage_planner/coverage.hpp"
#include "coverage_area_interface/srv/select_square.hpp"

namespace nav2_coverage_planner
{
  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(
        const nav2_costmap_2d::Costmap2D &costmap, const unsigned char * /*char_map*/,
        std::vector<nav2_costmap_2d::MapLocation> &cells)
        : costmap_(costmap), cells_(cells)
    {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset)
    {
      nav2_costmap_2d::MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const nav2_costmap_2d::Costmap2D &costmap_;
    std::vector<nav2_costmap_2d::MapLocation> &cells_;
  };
  class CoveragePlanner : public nav2_core::GlobalPlanner
  {
  public:
    CoveragePlanner() = default;
    ~CoveragePlanner() = default;

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

    bool transformPoseToAnotherFrame(const geometry_msgs::msg::PoseStamped &input_pose,
                                     geometry_msgs::msg::PoseStamped &transformed_pose, const std::string &goalFrame);

    bool transformPoseToGlobalFrame(
        const geometry_msgs::msg::PoseStamped &input_pose,
        geometry_msgs::msg::PoseStamped &transformed_pose);

  //  void mapToWorld(double mx, double my, double & wx, double & wy, nav2_costmap_2d::Costmap2D *costmap);
  //  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my, nav2_costmap_2d::Costmap2D *costmap);
    /**
     * @brief Set the corresponding cell cost to be free space
     * @param mx int of map X coordinate
     * @param my int of map Y coordinate
     */
    void clearRobotCell(unsigned int mx, unsigned int my);

    /**
     * @brief Determine if a new planner object should be made
     * @return true if planner object is out of date
     */
    bool isPlannerOutOfDate();

    void extractPlannerCostMap();

    void linearInterpolation(const geometry_msgs::msg::PoseStamped curr_start, const geometry_msgs::msg::PoseStamped cur_goal, nav_msgs::msg::Path &global_path);

    void receiveNewCoverageArea(const std::shared_ptr<coverage_area_interface::srv::SelectSquare::Request> request, // CHANGE
                                std::shared_ptr<coverage_area_interface::srv::SelectSquare::Response> response);

  private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;
    double transform_tolerance_{0}; ///< The timeout before transform errors
    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D *costmap_;

    nav2_costmap_2d::Costmap2D coverageMap;
    std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> plannnerMapPub;

    // The global frame of the costmap
    std::string global_frame_, name_, planner_frame_;

    rclcpp::Logger logger_{rclcpp::get_logger("CoveragePlanner")};

    double interpolation_resolution_;
    double xy_goal_tolerance;
    int planner_path_index;

    std::unique_ptr<Coverage> planner_;
    bool movedToOrigin;

    // service for receive the area to cover
    rclcpp::Service<coverage_area_interface::srv::SelectSquare>::SharedPtr coverageService;
    std::vector<double> coveragePose;

    // for checking if need to update planner's costmap
    
    bool needUpdateCoverageMap; 
    unsigned int costmap_last_x_cell, costmap_last_y_cell; // record the number of cell for the global costmap
                                                          // use for checking of global costmap has been update by map server
  };

} 

#endif // NAV2_COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_
