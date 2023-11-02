#ifndef NAV2_COSTMAP_2D_COSTMAP_2D_MODIFIED_HPP_
#define NAV2_COSTMAP_2D_COSTMAP_2D_MODIFIED_HPP_

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_costmap_2d
{

    // define a class tha inherits costmap_2d, but provide interface for
    class Costmap2DModified : public nav2_costmap_2d::Costmap2D
    {
    public:
        Costmap2DModified(
            unsigned int size_x, unsigned int size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value);

        /**
         * @brief Store all cell for a given rectangle shape.
         * 
         * @param lowerLeft The lowerLeft corner of the costmap.
         * @param upperLeft The upperLeft corner of the costmap.
         * @param lowerRight The lowerRight corner of the costmap.
         * @param upperRight The upperRight corner of the costmap.
         * @param areaLocation Vector that store all the cell location in rectangle define by lowerLeft, upperLeft, lowerRight & upperRight
         * @param polygon_x_size The row size in cell number
         * @param polygon_y_size The column size in cell number
         */
        void extractCostmap(const MapLocation &lowerLeft, const MapLocation &upperLeft,
                                     const MapLocation &lowerRight, const MapLocation &upperRight, std::vector<MapLocation> &areaLocation,
                                     unsigned int &polygon_x_size, unsigned int &polygon_y_size);
    };
}

#endif // NAV2_COSTMAP_2D_COSTMAP_2D_MODIFIED_HPP_
