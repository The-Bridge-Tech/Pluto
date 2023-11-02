#include "coverage.hpp"

using namespace nav2_straightline_planner;
void test1()
{
    int width = 2;
    int height = 2;
    Coverage cov = Coverage(width, height);
    cov.setupPathArray(2 * 2);
    int start[2];
    start[0] = start[1] = 0;
    cov.setStart(start);

    // only in debug
    unsigned char costmap[4] = {
        0, 0,
        0, 0};

    cov.setCostmap(costmap, true);
    cov.squareMovement();

    cov.savemap("test1.pgm");

    // check the value in costmap
    // for (auto i = 0; i < height; i++)
    // {
    //     for (auto j = 0; j < width; j++)
    //     {
    //         printf("cost %d, x:%d y:%d\n", cov.costarr[i * height + j], j, i);
    //     }
    // }

    // print out the path
    printf("Path size %d\n ", cov.npath);
    for (int i = 0; i < cov.npath; i++)
    {
        auto p = cov.pathArray[i];
        printf("coordinate X: %d y: %d\n", p.x, p.y);
        printf("        cost %d, x:%d y:%d\n", cov.costarr[cov.convertMapXYToIndex(p)], p.x, p.y);
    }
}

void test2()
{

    // test for
    int width = 3;
    int height = 4;
    Coverage cov = Coverage(width, height);
    cov.setupPathArray(width * height);
    int start[2];
    start[0] = 2;
    start[1] = 1;
    cov.setStart(start);

    // only in debug
    unsigned char costmap[12] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };

    cov.setCostmap(costmap, true);
    cov.squareMovement();

    cov.savemap("test2.pgm");

    // check the value in costmap
    // for(auto i = 0; i < height; i++){
    //     for(auto j = 0; j< width; j++){
    //         printf("cost %d, x:%d y:%d\n", cov.costarr[i*height + j],j,i );

    //     }
    // }

    // print out the path
    printf("Path size %d\n ", cov.npath);
    for (int i = 0; i < cov.npath; i++)
    {
        auto p = cov.pathArray[i];
        printf("coordinate X: %d y: %d\n", p.x, p.y);
        printf("        cost %d, x:%d y:%d\n", cov.costarr[cov.convertMapXYToIndex(p)], p.x, p.y);
    }
}

void test3()
{
    // test for 255(unknown value in the map)

    int width = 3;
    int height = 4;
    Coverage cov = Coverage(width, height);
    cov.setupPathArray(width * height);
    int start[2];
    start[0] = 0;
    start[1] = 0;
    cov.setStart(start);

    // only in debug
    unsigned char costmap[12] = {
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
    };

    cov.setCostmap(costmap, true);
    cov.squareMovement();

    cov.savemap("test3.pgm");

    // check the value in costmap
    // for(auto i = 0; i < height; i++){
    //     for(auto j = 0; j< width; j++){
    //         printf("cost %d, x:%d y:%d\n", cov.costarr[i*height + j],j,i );

    //     }
    // }

    // print out the path
    printf("Path size %d\n ", cov.npath);
    for (int i = 0; i < cov.npath; i++)
    {
        auto p = cov.pathArray[i];
        printf(" coordinate X: %d y: %d\n", p.x, p.y);
        printf("cost %d, x:%d y:%d\n", cov.costarr[cov.convertMapXYToIndex(p)], p.x, p.y);
    }
}

void test4()
{
    // test for with obstacle

    int width = 3;
    int height = 4;
    Coverage cov = Coverage(width, height);
    cov.setupPathArray(width * height);
    int start[2];
    start[0] = 0;
    start[1] = 0;
    cov.setStart(start);

    // only in debug
    unsigned char costmap[12] = {
        255,
        0,
        255,
        255,
        254,
        255,
        0,
        253,
        255,
        255,
        0,
        255,
    };

    cov.setCostmap(costmap, true);
    cov.squareMovement();

    cov.savemap("test4.pgm");

    // //check the value in costmap
    // for(auto i = 0; i < height; i++){
    //     for(auto j = 0; j< width; j++){
    //         printf("cost %d, x:%d y:%d\n", cov.costarr[i*height + j],j,i );

    //     }
    // }

    // print out the path
    printf("Path size %d\n ", cov.npath);
    for (int i = 0; i < cov.npath; i++)
    {
        auto p = cov.pathArray[i];
        printf("coordinate X: %d y: %d\n", p.x, p.y);
        printf("    cost %d, x:%d y:%d\n", cov.costarr[cov.convertMapXYToIndex(p)], p.x, p.y);
    }
}

void test5()
{
    // test for with obstacle

    int width = 3;
    int height = 4;
    Coverage cov = Coverage(width, height);
    cov.setupPathArray(width * height);
    int start[2];
    start[0] = 0;
    start[1] = 0;
    cov.setStart(start);

    // only in debug
    unsigned char costmap[12] = {
        0,
        0,
        0,
        0,
        0,
        0,
        254,
        0,
        0,
        0,
        0,
        0,
    };

    cov.setCostmap(costmap, true);
    cov.squareMovement();

    cov.savemap("test5.pgm");

    // //check the value in costmap
    // for(auto i = 0; i < height; i++){
    //     for(auto j = 0; j< width; j++){
    //         printf("cost %d, x:%d y:%d\n", cov.costarr[i*height + j],j,i );

    //     }
    // }

    // print out the path
    printf("Path size %d\n ", cov.npath);
    for (int i = 0; i < cov.npath; i++)
    {
        auto p = cov.pathArray[i];
        printf("coordinate X: %d y: %d\n", p.x, p.y);
        printf("    cost %d, x:%d y:%d\n", cov.costarr[cov.convertMapXYToIndex(p)], p.x, p.y);
    }
}
int main()
{

    test1();
    test2();
    test3();
    test4();
    test5(); // major case, no complete coverage
                // could be solve by backtrace
                // Did not implement backtrace, since the moving pattern of the roboot
                    // has a greater priority than complete coverage

    return 0;
}