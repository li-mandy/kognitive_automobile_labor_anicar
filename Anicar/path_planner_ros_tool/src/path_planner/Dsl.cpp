#include "Dsl.h"

#include <iostream>
const double Dsl::MAX_STEPS = 10000;

Dsl::Dsl(nav_msgs::OccupancyGrid map, geometry_msgs::Point start, geometry_msgs::Point goal) {

 _open_list.clear();
 _open_hash.clear();
 _path.clear();
 _km = 0;
 _map = new Map(map);
 _start = _map->getCell(start);
 if(_start->cost=Cell::COST_UNWALKABLE){
  _start = _map->getcloseWalkable(_start);
  ROS_INFO_STREAM("Start is unwalkable");
 }

 _goal = _map->getCell(goal);
 _last = _start;
 ROS_INFO_STREAM("Cost of Start is "<< _start->cost);
 ROS_INFO_STREAM("Cost of Goal is "<< _goal->cost);

 _rhs(_goal,0.0);

 _list_insert(_goal, std::pair<double,double>(_h(_start,_goal),0));

}

Dsl::~Dsl(){

}

void Dsl::_list_insert(Cell* u, std::pair<double,double> k)
{
 OL::iterator pos = _open_list.insert(OL_PAIR(k, u));
 _open_hash[u] = pos;
}

void Dsl::_list_remove(Cell* u)
{
 _open_list.erase(_open_hash[u]);
 _open_hash.erase(_open_hash.find(u));
}

void Dsl::_list_update(Cell* u, std::pair<double,double> k)
{
 OL::iterator pos1 = _open_hash[u];
 OL::iterator pos2 = pos1;

 if (pos1 == _open_list.end())
 {
  pos2 = _open_list.end();
 }
 else
 {
  pos2++;
 }

 _open_list.erase(pos1);
 _open_hash[u] = _open_list.insert(pos2, OL_PAIR(k, u));
}

double Dsl::_g(Cell* u, double value)
{
 _cell(u);
 std::pair<double,double>* g_rhs = &_cell_hash[u];

 if (value != DBL_MIN)
 {
  g_rhs->first = value;
 }

 return g_rhs->first;
}

double Dsl::_h(Cell* a, Cell* b)
{
    double dx = a->position().x - b->position().x;
    double dy = a->position().y - b->position().y;



    return sqrt(dx*dx + dy*dy);
//
// geometry_msgs::Point posa = a->position();
// geometry_msgs::Point posb = b->position();
// unsigned int min = labs(posa.x - posb.x);
// unsigned int max = labs(posa.y - posb.y);
//
// if (min > max)
// {
//  unsigned int tmp = min;
//  min = max;
//  max = tmp;
// }
//
// return ((Math::SQRT2 - 1.0) * min + max);
}

std::pair<double,double> Dsl::_k(Cell* u)
{
 double g = _g(u);
 double rhs = _rhs(u);
 double min = (g < rhs) ? g : rhs;
 return std::pair<double,double>((min + _h(_start, u) + _km), min);
}

double Dsl::_rhs(Cell* u, double value)
{
 if (u == _goal)
  return 0;

 _cell(u);
 std::pair<double,double>* g_rhs = &_cell_hash[u];

 if (value != DBL_MIN)
 {
  g_rhs->second = value;
 }

 return g_rhs->second;
}

bool Dsl::KeyCompare::operator()(const std::pair<double,double>& p1, const std::pair<double,double>& p2) const
{
 if (Math::less(p1.first, p2.first))				return true;
 else if (Math::greater(p1.first, p2.first))		return false;
 else if (Math::less(p1.second,  p2.second))		return true;
 else if (Math::greater(p1.second, p2.second))	return false;
 return false;
}

nav_msgs::Path Dsl::path()
{
 nav_msgs::Path myPath;
 myPath.header.frame_id = "world";
 myPath.header.stamp = ros::Time::now();


for( std::list<Cell*>::iterator it = _path.begin();it != _path.end();it++){
 geometry_msgs::PoseStamped myPose;
 myPose.header.frame_id="world";
 myPose.header.stamp = ros::Time::now();
 myPose.pose.position = (*it)->position();
 myPath.poses.push_back(myPose);

}



 return myPath;
}

Cell* Dsl::goal(Cell* u)
{

 //todo use position
 if (u == NULL)
  return _goal;

 // Hack implementation
 _goal = u;

 return _goal;
}
void Dsl::setnewStart(geometry_msgs::Point start){
    _start = _map->getCell(start);
    ROS_INFO_STREAM("Cost of Start is "<< _start->cost);
}
void Dsl::setnewGoal(geometry_msgs::Point goal){
    _goal = _map->getCell(goal);
    ROS_INFO_STREAM("Cost of Goal is "<< _goal->cost);
}
Cell* Dsl::start(Cell* u)
{
 //todo use position
 if (u == NULL)
  return _start;

 _start = u;

 return _start;
}

bool Dsl::replan()
{
 _path.clear();

 bool result = _compute();

 // Couldn't find a solution
 if ( !result)
  return false;

 Cell* current = _start;
 _path.push_back(current);

 // Follow the path with the least cost until goal is reached
 while (current != _goal)
 {
  if (current == NULL || _g(current) == Math::INF){
      return false;
  }


  current = _min_succ(current).first;

  _path.push_back(current);
 }



 return true;
}

void Dsl::reset(geometry_msgs::Point point){
    Cell* myCell = _map->getCell(point);
    update(myCell,myCell->oldCost);
}

void Dsl::update(geometry_msgs::Point point,double cost){
    update(_map->getCell(point),cost);

}

void Dsl::update(Cell* u, double cost)
{
 if (u == _goal)
  return;

 // Update km
 _km += _h(_last, _start);
 _last = _start;

 _cell(u);

 double cost_old = u->cost;
 double cost_new = cost;
 u->cost = cost;

 Cell** nbrs = u->nbrs();

 double tmp_cost_old, tmp_cost_new;
 double tmp_rhs, tmp_g;

 // Update u
 for (unsigned int i = 0; i < Cell::NUM_NBRS; i++)
 {
  if (nbrs[i] != NULL)
  {
   u->cost = cost_old;
   tmp_cost_old = _cost(u, nbrs[i]);
   u->cost = cost_new;
   tmp_cost_new = _cost(u, nbrs[i]);

   tmp_rhs = _rhs(u);
   tmp_g = _g(nbrs[i]);

   if (Math::greater(tmp_cost_old, tmp_cost_new))
   {
    if (u != _goal)
    {
     _rhs(u, std::min(tmp_rhs, (tmp_cost_new + tmp_g)));
    }
   }
   else if (Math::equals(tmp_rhs, (tmp_cost_old + tmp_g)))
   {
    if (u != _goal)
    {
     _rhs(u, _min_succ(u).second);
    }
   }
  }
 }

 _update(u);

 // Update neighbors
 for (unsigned int i = 0; i < Cell::NUM_NBRS; i++)
 {
  if (nbrs[i] != NULL)
  {
   u->cost = cost_old;
   tmp_cost_old = _cost(u, nbrs[i]);
   u->cost = cost_new;
   tmp_cost_new = _cost(u, nbrs[i]);

   tmp_rhs = _rhs(nbrs[i]);
   tmp_g = _g(u);

   if (Math::greater(tmp_cost_old, tmp_cost_new))
   {
    if (nbrs[i] != _goal)
    {
     _rhs(nbrs[i], std::min(tmp_rhs, (tmp_cost_new + tmp_g)));
    }
   }
   else if (Math::equals(tmp_rhs, (tmp_cost_old + tmp_g)))
   {
    if (nbrs[i] != _goal)
    {
     _rhs(nbrs[i], _min_succ(nbrs[i]).second);
    }
   }

   _update(nbrs[i]);
  }
 }
}

void Dsl::_update(Cell* u)
{
 bool diff = _g(u) != _rhs(u);
 bool exists = (_open_hash.find(u) != _open_hash.end());

 if (diff && exists)
 {
  _list_update(u, _k(u));
 }
 else if (diff && ! exists)
 {
  _list_insert(u, _k(u));
 }
 else if ( ! diff && exists)
 {
  _list_remove(u);
 }
}

double Dsl::_cost(Cell* a, Cell* b)
{
 //todo
 if (a->cost == Cell::COST_UNWALKABLE || b->cost == Cell::COST_UNWALKABLE)
  return Cell::COST_UNWALKABLE;

 double dx = a->position().x - b->position().x;
 double dy = a->position().y - b->position().y;

 //double dboarder = b->position().y;
//double dboarder = 1;
// for(unsigned int i = 0;i<Cell::NUM_NBRS;i++){
//  if(a->nbrs()[i] == b){
//   int nbrmap[] = {5,3,0,6,1,7,4,2};
//   int nbr = nbrmap[i];
//
//    Cell* search = a;
//    geometry_msgs::Point pos;
//    int interupt = 0;
//    do{
//     interupt++;
//     pos = search->position();
//     search = search->nbrs()[nbr];
//    }
//    while(!(search == NULL || search->cost == Cell::COST_UNWALKABLE) && interupt < 20);
//
//    double dbx = a->position().x - pos.x;
//    double dby = a->position().y - pos.y;
//    dboarder = sqrt(dbx*dbx + dby*dby)*3;
//  }
// }


// if ((dx + dy) > 1)
// {
//  scale = Math::SQRT2;
// }

 //return scale * ((a->cost + b->cost) / 2);
 return sqrt(dx*dx + dy*dy)*(a->cost+b->cost)/2;
}

bool Dsl::_compute()
{
 if (_open_list.empty())
  return false;

 KeyCompare key_compare;

 int attempts = 0;

 Cell* u;
 std::pair<double,double> k_old;
 std::pair<double,double> k_new;
 Cell** nbrs;
 double g_old;
 double tmp_g, tmp_rhs;

 while (( ! _open_list.empty() && key_compare(_open_list.begin()->first, _k(_start))) || !Math::equals(_rhs(_start), _g(_start)))
 {
  // Reached max steps, quit
  if (++attempts > Dsl::MAX_STEPS){
      ROS_INFO_STREAM("reached max steps");
      return false;
  }


  u = _open_list.begin()->second;
  k_old = _open_list.begin()->first;
  k_new = _k(u);

  tmp_rhs = _rhs(u);
  tmp_g = _g(u);

  if (key_compare(k_old, k_new))
  {
   _list_update(u, k_new);
  }
  else if (Math::greater(tmp_g, tmp_rhs))
  {
   _g(u, tmp_rhs);
   tmp_g = tmp_rhs;

   _list_remove(u);

   nbrs = u->nbrs();

   for (unsigned int i = 0; i < Cell::NUM_NBRS; i++)
   {
    if (nbrs[i] != NULL)
    {
     if (nbrs[i] != _goal)
     {
      _rhs(nbrs[i], std::min(_rhs(nbrs[i]), _cost(u, nbrs[i]) + tmp_g));
     }

     _update(nbrs[i]);
    }
   }
  }
  else
  {
   g_old = tmp_g;
   _g(u, Math::INF);

   // Perform action for u
   if (u != _goal)
   {
    _rhs(u, _min_succ(u).second);
   }

   _update(u);

   nbrs = u->nbrs();

   // Perform action for neighbors
   for (unsigned int i = 0; i < Cell::NUM_NBRS; i++)
   {
    if (nbrs[i] != NULL)
    {
     if (Math::equals(_rhs(nbrs[i]), (_cost(u, nbrs[i]) + g_old)))
     {
      if (nbrs[i] != _goal)
      {
       _rhs(nbrs[i], _min_succ(nbrs[i]).second);
      }
     }

     _update(nbrs[i]);
    }
   }
  }
 }

 return true;
}

void Dsl::_cell(Cell* u)
{
 if (_cell_hash.find(u) != _cell_hash.end())
  return;

 double h = Math::INF;
 _cell_hash[u] = std::pair<double,double>(h, h);
}

std::pair<Cell*,double> Dsl::_min_succ(Cell* u)
{
 Cell** nbrs = u->nbrs();

 double tmp_cost, tmp_g;

 Cell* min_cell = NULL;
 double min_cost = Math::INF;

 for (unsigned int i = 0; i < Cell::NUM_NBRS; i++)
 {
  if (nbrs[i] != NULL)
  {
   tmp_cost = _cost(u, nbrs[i]);
   tmp_g = _g(nbrs[i]);

   if (tmp_cost == Math::INF || tmp_g == Math::INF)
    continue;

   tmp_cost += tmp_g;

   if (tmp_cost < min_cost)
   {
    min_cell = nbrs[i];
    min_cost = tmp_cost;
   }
  }
 }

 return std::pair<Cell*,double>(min_cell, min_cost);
}