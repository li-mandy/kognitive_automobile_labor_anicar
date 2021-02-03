#include "Cell.h"


const unsigned int Cell::NUM_NBRS = 8;
const double Cell::COST_UNWALKABLE = DBL_MAX;

Cell::Cell(geometry_msgs::Point pos, double cost){
    _init = false;
    _nbrs = NULL;
    _position = pos;
    this->cost = cost;
    oldCost = cost;

}

Cell::~Cell(){


}


void Cell::init(Cell** nbrs){
    if(_init)
        return;

    _init = true;

    _nbrs = nbrs;
}

Cell** Cell::nbrs() {
    return _nbrs;
}

geometry_msgs::Point Cell::position() {

    return _position;
}

const int Cell::Hash::C = 1000000;

size_t Cell::Hash::operator()(Cell* c) const
{   geometry_msgs::Point pos = c->position();
    return Cell::Hash::C * pos.x + pos.y;
}