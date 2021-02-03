#ifndef MAP_H
#define MAP_H

#include "Cell.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

class Map {

public:
    Map(nav_msgs::OccupancyGrid map);
    Cell* operator()(const unsigned int row, const unsigned int col);
    bool has(unsigned int row, unsigned int col);
    uint32_t cols();
    uint32_t rows();
    Cell* getCell(geometry_msgs::Point point);
    Cell* getcloseWalkable(Cell*);


protected:
    Cell*** _cells;
    nav_msgs::OccupancyGrid _map;




};

#endif //MAP_H