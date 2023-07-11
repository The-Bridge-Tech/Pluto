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
    class RectangleLineCells
    {
    public:
        RectangleLineCells(
            const Costmap2D &costmap,
            std::vector<MapLocation> &cells)
            : costmap_(costmap), cells_(cells)
        {
        }

        // just push the relevant cells back onto the list
        inline void operator()(unsigned int offset)
        {
            MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            cells_.push_back(loc);
        }

    private:
        const Costmap2D &costmap_;
        std::vector<MapLocation> &cells_;
    };
    // define a class tha inherits costmap_2d, but provide interface for
    class Costmap2DModified : public nav2_costmap_2d::Costmap2D
    {
    public:
        Costmap2DModified(
            unsigned int size_x, unsigned int size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value);

        // the only different is that it will add a method


        // void extractCostmap(

        //     const std::vector<MapLocation> &polygon,

        //     std::vector<MapLocation> &areaLocation,
        //     unsigned int &polygon_x_size, unsigned int &polygon_y_size);
        void extractCostmap(const MapLocation &lowerLeft, const MapLocation &upperLeft,
                                     const MapLocation &lowerRight, const MapLocation &upperRight, std::vector<MapLocation> &areaLocation,
                                     unsigned int &polygon_x_size, unsigned int &polygon_y_size);
    };
}

#endif // NAV2_COSTMAP_2D_COSTMAP_2D_MODIFIED_HPP_
