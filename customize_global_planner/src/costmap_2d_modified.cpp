

#include "nav2_costmap_2d/costmap_2d.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include <stdexcept>
#include <cmath>

#include "customize_global_planner/costmap_2d_modified.hpp"
namespace nav2_costmap_2d
{
    Costmap2DModified::Costmap2DModified(
        unsigned int size_x, unsigned int size_y, double resolution,
        double origin_x, double origin_y, unsigned char default_value)
    {

        // create the costmap
        costmap_ = NULL;
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;

        initMaps(size_x, size_y);
    }

    void Costmap2DModified::extractCostmap(const MapLocation &lowerLeft, const MapLocation &upperLeft,
                                           const MapLocation &lowerRight, const MapLocation &upperRight, std::vector<MapLocation> &areaLocation,
                                           unsigned int &polygon_x_size, unsigned int &polygon_y_size)
    {

     
        std::vector<nav2_costmap_2d::MapLocation> left_side;
        std::vector<nav2_costmap_2d::MapLocation> right_side;

        PolygonOutlineCells right_cell_gatherer(*this, costmap_, right_side);
        PolygonOutlineCells left_cell_gatherer(*this, costmap_, left_side);
        auto start = lowerLeft;
        auto end = upperLeft;
        raytraceLine(left_cell_gatherer, start.x, start.y, end.x, end.y);   // first, get the left boundary

        start = lowerRight;
        end = upperRight;
        raytraceLine(right_cell_gatherer, start.x, start.y, end.x, end.y); // then get the right side

        std::size_t xSize = std::min(left_side.size(), right_side.size());
        std::size_t ySize = 0;

        for (std::size_t i = 0; i < xSize; i++)
        {
            std::vector<nav2_costmap_2d::MapLocation> temp;
            PolygonOutlineCells temp_gather(*this, costmap_, temp); // find lines from left to right 

            MapLocation s = left_side.at(i);
            MapLocation e = right_side.at(i);
            raytraceLine(temp_gather, s.x, s.y, e.x, e.y);

            ySize = std::max(temp.size(), ySize);

            for (size_t j = 0; j < temp.size(); j++)
            {

                MapLocation loc = temp[j];

                areaLocation.push_back(loc);
            }
        }
        polygon_x_size = xSize;
        polygon_y_size = ySize;
    }

}