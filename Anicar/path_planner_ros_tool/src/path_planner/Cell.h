#ifndef CELL_H
#define CELL_H

#include <geometry_msgs/Point.h>
#include <functional>
#include <stdlib.h>

#include "Math.h"

class Cell {

public:

    class Hash : public std::unary_function<Cell*, size_t>
    {
    public:

        static const int C;

        size_t operator()(Cell* c) const;
    };

    static const unsigned int NUM_NBRS;
    static const double COST_UNWALKABLE;



    Cell(geometry_msgs::Point pos, double cost = 1.0);
    ~Cell();
    Cell** nbrs();
    geometry_msgs::Point position();
    void init(Cell** nbrs);
    double cost;
    double oldCost;

protected:
    bool _init;
    geometry_msgs::Point _position;
    Cell** _nbrs;


};

#endif //CELL_H