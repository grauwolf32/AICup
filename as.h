#ifndef _ASTAR_H_
#define _ASTAR_H_
#include "TileType.h"

typedef std::vector<std::pair<int,int> > path;
typedef std::pair<int,int> cell;
typedef std::vector<std::vector<model::TileType> > tilemap;
typedef std::vector<std::vector<double> > costmap;

double euclidian_dist(int x,int y);
double manhatann_dist(int x,int y);
double euclidian_dist(cell x,cell y);
double manhatann_dist(cell x,cell y);

cell findMinF(std::set<cell >& wawe, costmap& F);
double transition_cost(cell last,cell current,cell next);
void getUnclosedNeighbours(cell current,tilemap& map,std::set<cell >& closed,std::set<cell >& neighbours);
int findPathA(cell start,cell end,path& best_path,tilemap& t_map, double (*h)(cell x,cell y) );

#endif
