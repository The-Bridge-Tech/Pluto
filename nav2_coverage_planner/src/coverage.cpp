

#include <algorithm>

// #define ROS_MODE true
// #if ROS_MODE
#include "rclcpp/rclcpp.hpp"
#include "nav2_coverage_planner/coverage.hpp"
// #else
//     #include "coverage.hpp"
//     #include <iostream>
// #endif

namespace nav2_coverage_planner
{
    Coverage::Coverage(int xs, int ys)
    {
        // create cell arrays
        costarr = NULL;

        setNavArr(xs, ys);

        // start
        start[0] = start[1] = 0;

        // path buffers
        npathbuf = npath = 0;
        pathArray = NULL;
    }

    Coverage::~Coverage()
    {
        if (costarr)
        {
            delete[] costarr;
        }
        if (pathArray)
        {
            delete[] pathArray;
        }
    }

    bool
    Coverage::setStart(int *g)
    {

        // The algorithm require the start point be on the edge of the costmap
        auto x = g[0];
        auto y = g[1];

        if (true) // if (x == 0 || y == 0 || x == this->nx - 1 || y == this->ny - 1)
        {         // remember the offset by 1 index
            start[0] = g[0];
            start[1] = g[1];
            return true;
        }
        else
        {
#if ROS_MODE
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[Coverage] the start point need to be on the edge of the costmap");
#else
            std::cout << "[Coverage] the start point need to be on the edge of costmap" << std::endl;
#endif
        }

#if ROS_MODE
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp"), "[Coverage] Setting start to %d,%d\n", start[0],
            start[1]);
#else
        std::cout << "[Coverage] Setting start to " << start[0] << " " << start[1] << std::endl;
#endif
    }

    //
    // Set/Reset map size
    //

    void
    Coverage::setNavArr(int xs, int ys)
    {
#if ROS_MODE

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[Coverage] Array is %d x %d\n", xs, ys);
#else
        std::cout << "[Coverage] Array is " << xs << " * " << ys << std::endl;
#endif
        nx = xs;
        ny = ys;
        ns = nx * ny;

        if (costarr)
        {
            delete[] costarr;
        }

        costarr = new COSTTYPE[ns]; // cost array, 2d config space
        memset(costarr, 0, ns * sizeof(COSTTYPE));
    }

    //
    // set up cost array, usually from ROS
    //

    void
    Coverage::setCostmap(const COSTTYPE *cmap, bool isROS)
    {
        // for the costmap
        // view all area according to ros standard
        //
        COSTTYPE *cm = costarr;
        if (isROS)
        { // ROS-type cost array
            for (int i = 0; i < ny; i++)
            {
                int k = i * nx;
                for (int j = 0; j < nx; j++, k++, cmap++, cm++)
                {

                    int v = *cmap;
                    *cm = v;
                    // as of right now, keep all Costmap value as it is in ros2, since we assume no static obstacle in the map
                    // TODO: somehow update the cost in map  to the algorithm requirement if needed.
                }
            }
        }
    }

    mapXY *Coverage::getPathXY() { return this->pathArray; }
    int Coverage::getPathLen() { return npath; }

    int Coverage::convertMapXYToIndex(const mapXY &p)
    {
        return p.x + (p.y * nx);
    }

    bool Coverage::moveUp(mapXY &p)
    {

        // // note: take into consider that index start a 0;
        if (p.y == 0)
        {
            return false;
        }
        else
        {
            p.y -= 1;
            return true;
        }
    }

    bool Coverage::moveDown(mapXY &p)
    {

        // // check if k is already at the very last row
        if (p.y + 1 == ny)
        {
            return false;
        }
        else
        {
            p.y += 1;
            return true;
        }
    }

    bool Coverage::moveLeft(mapXY &p)
    {
        // check if k is already at the very left of the matrix grid
        if (p.x == 0)
        {
            return false;
        }
        else
        {
            p.x -= 1;
            return true;
        }
    }
    bool Coverage::moveRight(mapXY &p)
    {
        // check if k is already at the very right of the matrix grid
        if (p.x + 1 == nx)
        {
            return false;
        }
        else
        {
            p.x += 1;
            return true;
        }
    }

    //
    // Path construction
    // Find gradient at array points, interpolate path
    // Use step size of pathStep, usually 0.5 pixel
    //
    // Some sanity checks:
    //  1. Stuck at same index position
    //  2. Doesn't get near goal
    //  3. Surrounded by high potentials
    //

    void Coverage::setupPathArray(int n)
    {
        if (npathbuf < n)
        {
            if (pathArray)
            {
                delete[] pathArray;
            }

            pathArray = new mapXY[n];
            npathbuf = n;
            // if (pathx)
            // {
            //     delete[] pathx;
            // }
            // if (pathy)
            // {
            //     delete[] pathy;
            // }
            // pathx = new float[n];
            // pathy = new float[n];
            // npathbuf = n;
        }
        npath = 0;
    }
    // int
    // Coverage::calcPath()
    // {
    //     // test write
    //     // savemap("test");

    //     // check path arrays, initialize if necessary

    //     int n = this->nx * this->ny;
    //     if (npathbuf < n)
    //     {
    //         if (pathx)
    //         {
    //             delete[] pathx;
    //         }
    //         if (pathy)
    //         {
    //             delete[] pathy;
    //         }
    //         pathx = new float[n];
    //         pathy = new float[n];
    //         npathbuf = n;
    //     }

    //     // set up offset
    //     float dx = 0;
    //     float dy = 0;
    //     npath = 0;

    //     // // go for <n> cycles at most
    //     // for (int i = 0; i < n; i++)
    //     // {

    //     // }

    //     //  return npath;  // out of cycles, return failure
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
    //     // savemap("navfn_pathlong");
    //     return 0; // out of cycles, return failure
    // }

    bool Coverage::isObstacle(mapXY &p)
    {
        auto value = this->costarr[convertMapXYToIndex(p)];
        if (value < inscribedInflatedObstacle || value == unknownValue)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void Coverage::markVisited(mapXY &p)
    {

        int index = convertMapXYToIndex(p);

        this->costarr[index] = lethalObstacle; // nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    bool Coverage::findNearbyFreeSpace(mapXY &p, char &direction)
    {
        mapXY cur_pos = p;

        char currentDir = direction;

        bool hasFound = false;
        for (char i = 0; i < 4 && !hasFound; i++)
        {
            switch (currentDir)
            {

            case Up:
            {
                // check if can move up
                if (moveUp(cur_pos) && !(isObstacle(cur_pos)))
                {
                    // means is free space
                    p.x = cur_pos.x;
                    p.y = cur_pos.y;
                    hasFound = true;
                }
                else
                {
                    cur_pos = p; // reset position, see if can move right
                    currentDir = Right;
                }
                break;
            }
            case Right:
            {
                if (moveRight(cur_pos) && !(isObstacle(cur_pos)))
                {
                    // means is free space
                    p.x = cur_pos.x;
                    p.y = cur_pos.y;
                    hasFound = true;
                }
                else
                {
                    cur_pos = p;
                    currentDir = Down;
                }
                break;
            }
            case Down:
            {
                // check if can move bottom
                if (moveDown(cur_pos) && !(isObstacle(cur_pos)))
                {
                    // means is free space
                    p.x = cur_pos.x;
                    p.y = cur_pos.y;
                    hasFound = true;
                }
                else
                {
                    cur_pos = p;
                    currentDir = Left;
                }
                break;
            }
            case Left:
            {
                // check if can move left
                if (moveLeft(cur_pos) && !(isObstacle(cur_pos)))
                {
                    // means is free space
                    p.x = cur_pos.x;
                    p.y = cur_pos.y;
                    hasFound = true;
                }
                else
                {
                    cur_pos = p; // reset position, check for up
                    currentDir = Up;
                }
                break;
            }
            }
        }

        // TODO: in the future
        //  IF we decide that the way how we cut it is not the first priority
        //  could add something similar to the idea of backtrace
        //  If hasFound == False, means not more valid movement at this point?
        //  basically, go back to the previous point (the latest point in pathArray), and see if it could have other way around
        direction = currentDir;
        return hasFound;
    }
    void Coverage::squareMovement()
    {
        // only n times,

        // moving direction
        // up -> right > down -> left

        int totalCoverageSize = nx * ny;
        // check if overflow
        if (nx != 0 && (totalCoverageSize / nx) != ny)
        {
#if ROS_MODE
            RCLCPP_ERROR(rclcpp::get_logger("Coverage"), "The map size is too big. Consider decrease given coverage size in map");
#else
            std::cout << "The map size is too big. Consider decrease given coverage size in map" << std::endl;
#endif
            return;
        }

        mapXY startPos = {start[0], start[1]}; // this->start[0] + this->start[1] * nx;

        char direction = Up;
        // only change direction if
        // 1. the previous direction is moveable. Example if we are moving up, change to left it is empty and movable
        // 2. change to next direction if has obstacle in front
        mapXY currentPos = startPos;
        // mapXY lastPost = startPos;

        mapXY previousDirectionPosition = startPos;
        if (!moveLeft(previousDirectionPosition))
        {
            // means failed to move down
            ;
        }; // previous direction movement

        // loop assume currentPos always is valid

        bool hasPath = true;
        while (hasPath)
        {

            // first add the current pose to path
            pathArray[npath] = currentPos;
            npath++;
            markVisited(currentPos);

            // then determine which direction we should move next

            hasPath = findNearbyFreeSpace(currentPos, direction);
            // // 1. check the previous direction see if valid
            // mapXY temp = currentPos;
            // moveLeft(temp);
            // if (!(isObstacle(temp)))
            // {
            //     // change heading back to left
            //     currentPos = temp;
            //     direction = Left;
            // }
            // else
            // {
            //     /// find new path from this case
            //     hasPath = findNearbyFreeSpace(currentPos, Up);
            // }
            //}
        }
    }

    //
    // debug writes
    // saves costmap and start/goal
    //

    void
    Coverage::savemap(const char *fname)
    {
        char fn[4096];

#if ROS_MODE
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[Coverage] Saving costmap and start/goal points");
#endif
        // write start and goal points
        snprintf(fn, sizeof(fn), "%s.txt", fname);
        FILE *fp = fopen(fn, "w");
        if (!fp)
        {
#if ROS_MODE
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
#else
            std::cout << "Cannot open file " << fname << std::endl;
#endif
            return;
        }
        fprintf(fp, "Start: %d %d\n", start[0], start[1]);
        fclose(fp);

        // write cost array
        if (!costarr)
        {
            return;
        }
        snprintf(fn, sizeof(fn), "%s.pgm", fname);
        fp = fopen(fn, "wb");
        if (!fp)
        {
#if ROS_MODE
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
#else
            std::cout << "Cannot ope file " << fname << std::endl;
#endif
            return;
        }
        fprintf(fp, "P5\n%d\n%d\n%d\n", nx, ny, 0xff);
        fwrite(costarr, 1, nx * ny, fp);
        fclose(fp);
    }

}
