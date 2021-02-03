#ifndef DSL_H
#define DSL_H

#include "Map.h"
#include "Math.h"
#include "Cell.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <list>
#include <map>
#ifdef WIN32
#include <unordered_map>
#else
#include <tr1/unordered_map>

#endif

class Dsl {

public:

    struct KeyCompare : public std::binary_function<std::pair<double,double>, std::pair<double,double>, bool>
    {
        bool operator()(const std::pair<double,double>& p1, const std::pair<double,double>& p2) const;
    };
    static const double MAX_STEPS;
    Dsl(nav_msgs::OccupancyGrid map, geometry_msgs::Point start, geometry_msgs::Point goal);
    ~Dsl();
    nav_msgs::Path path();

    Cell* goal(Cell* u = NULL);
    Cell* start(Cell* u = NULL);

    bool replan();
    void update(Cell* u, double cost);
    void reset(geometry_msgs::Point);
    void update(geometry_msgs::Point,double cost);
    void setnewStart(geometry_msgs::Point);
    void setnewGoal(geometry_msgs::Point);

protected:
    Map *_map;
    double _km;
    Cell *_start, *_goal, *_last;

    typedef std::pair<std::pair<double,double>, Cell*> OL_PAIR;
    typedef std::multimap<std::pair<double,double>, Cell*, KeyCompare> OL;
    OL _open_list;

    typedef std::unordered_map<Cell*, OL::iterator, Cell::Hash> OH;
    OH _open_hash;

    typedef std::unordered_map<Cell*, std::pair<double,double>, Cell::Hash> CH;
    CH _cell_hash;

    std::list<Cell*> _path;
    void _list_insert(Cell* u, std::pair<double,double> k);
    void _list_remove(Cell* u);
    void _list_update(Cell* u, std::pair<double,double> k);
    double _rhs(Cell* u, double value = DBL_MIN);

    double _g(Cell* u, double value = DBL_MIN);
    double _h(Cell* a, Cell* b);
    std::pair<double,double> _k(Cell* u);

    bool _compute();
    double _cost(Cell* a, Cell* b);

    std::pair<Cell*,double> _min_succ(Cell* u);
    void _update(Cell* u);

    void _cell(Cell* u);
};

#endif //DSL_H
