

#ifndef NAV2_COVERAGE_PLANNER_COVERAGE_HPP_
#define NAV2_COVERAGE_PLANNER_COVERAGE_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nav2_costmap_2d/cost_values.hpp"
// 1. use 4 points to get grid size in box x, y (might involve resizing to ensure a perfect gps points)
// 2. calculate a yaw offset
// 3. create grid base on given x, y
namespace customize_global_planner
{

#define ROS_MODE true
#if ROS_MODE
#define unknownValue nav2_costmap_2d::NO_INFORMATION
#define inscribedInflatedObstacle nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE
#define lethalObstacle nav2_costmap_2d::LETHAL_OBSTACLE

#else
#define unknownValue 255 // https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp
#define inscribedInflatedObstacle 253
#define lethalObstacle 254
#endif

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char // Whatever is used...
#endif

  // The setup of costmap in ROS2

  /**
   * @brief X is the column, Y is the row of 2D map array
   *
   */
  typedef struct mapXY
  {
    int x;
    int y;
  } mapXY_t;

// for the movement direction
#define Up 0
#define Left 1
#define Right 2
#define Down 3

  /**
   * @class Coverage
   * @brief Navigation function class. Holds buffers for costmap, navfn map. Maps are pixel-based.
   *  Origin is upper left, x is right, y is down.
   */
  class Coverage
  {
  public:
    /**
     * @brief  Constructs the planner
     * @param nx The x size of the map
     * @param ny The y size of the map
     */
    Coverage(int nx, int ny);

    ~Coverage();

    /**
     * @brief  Sets or resets the size of the map
     * @param nx The x size of the map
     * @param ny The y size of the map
     */
    void setNavArr(int nx, int ny);
    int nx, ny, ns; /**< size of grid, in pixels */

    /**
     * @brief  Set up the cost array for the planner, usually from ROS
     * @param cmap The costmap
     * @param isROS Whether or not the costmap is coming in in ROS format
     */
    void setCostmap(const COSTTYPE *cmap, bool isROS = true);

    /**
     * @brief Get the pathArray, which contains the complete coverage path.
     *
     * @return mapXY*
     */
    mapXY *getPathXY();

    /**
     * @brief  Accessor for the length of a path
     * @return The length of a path, 0 if not found
     */
    int getPathLen();

    /** cell arrays */
    COSTTYPE *costarr; /**< cost array in 2D configuration space >*/

    /**
     * @brief  Sets the start position for the planner.
     * @param start the start position
     */
    bool setStart(int *start);
    // contain the x, y of the start
    int start[2];

    /**  paths */
    mapXY *pathArray;
    int npath;    /**< number of path points */
    int npathbuf; /**< size of pathx, pathy buffers */


    /**
     * @brief Convert MapXY to correspond index in costarr;
     * 
     * @param p 
     * @return int the index of the cell in costarr;
     */
    int convertMapXYToIndex(const mapXY &p);


    bool moveUp(mapXY &p);
    bool moveDown(mapXY &p);
    bool moveRight(mapXY &p);
    bool moveLeft(mapXY &p);

    void setupPathArray(int n);

    bool isObstacle(mapXY &p);
    void markVisited(mapXY &p);

    void squareMovement();
    bool findNearbyFreeSpace(mapXY &p, char &direction);

    /** save costmap */
    /**< write out costmap and start/goal states as fname.pgm and fname.txt */
    void savemap(const char *fname);
  };

} // namespace

#endif //
