#include <utility>
#include <vector>
#include <iostream>
#include <set>
#include <map>
#include <math.h>
#include "TileType.h"
#include "as.h"

const int infinite = 0x7FFFFFFF;
const int dist_coeff = 20;

int findPathA(cell start,cell end,path& best_path,tilemap& t_map, double (*h)(cell x,cell y) )
{
	int width = 0;
	int height = 0;
	int is_ok = 0;
	cell current;
	
	width = t_map[0].size();
	height = t_map.size();

	costmap G(height,std::vector<double>(width, infinite));
	costmap F(height,std::vector<double>(width, infinite));

	G[start.second][start.first] = 0;
	F[start.second][start.first] = G[start.second][start.first] + dist_coeff*h(start,end);

	std::set<cell > wawe;
	std::set<cell > closed;
	std::map<cell,cell > from;
	std::set<cell > unclosedNeighbours; 
	
	from[start] = start;
	wawe.insert(start);

	while(!wawe.empty())
	{
		current = findMinF(wawe,F);
		if(current == end){is_ok = 1;break;}

		wawe.erase(current);
		closed.insert(current);
		getUnclosedNeighbours(current,t_map,closed,unclosedNeighbours);
		for(auto it = unclosedNeighbours.begin();it != unclosedNeighbours.end();it++)
		{
			double temp_G = G[current.second][current.first] + transition_cost(from[current],current,*it);
			if(wawe.count(*it) == 0 || temp_G < G[it->second][it->first])
			{
				from[*it] = current;
				G[it->second][it->first] = temp_G;
				F[it->second][it->first] = G[it->second][it->first] + dist_coeff*h(*it,end);
				wawe.insert(*it);
			}
		}
	}

	if(!is_ok) return -1;
	
	best_path.clear();
	best_path.push_back(end);
	
	while(current != start)
	{
		best_path.push_back(from[current]); 
		current = from[current];
	}

	best_path = std::vector<cell >(best_path.rbegin(),best_path.rend());
	return 0;
}

double euclidian_dist(int x,int y)
{
	return (double)sqrt((float)(x*x + y*y));
}

double manhatann_dist(int x,int y)
{
	return (double)(fabs(x) + fabs(y));
}

double euclidian_dist(cell x,cell y)
{
	return euclidian_dist(y.first - x.first,y.second - x.second);
}
double manhatann_dist(cell x,cell y)
{
	return manhatann_dist(y.first - x.first,y.second - x.second);
}

double transition_cost(cell last,cell current,cell next)
{
	if(fabs(next.first - last.first) > 0 && fabs(next.second - last.second) > 0)
		return 2.0;
	return 1.0;
}


cell findMinF(std::set<cell >& wawe, costmap& F)
{
	int min = infinite;
	cell      ans(-1,-1);

	for(auto it = wawe.begin();it != wawe.end();it++)
	{
		if(min > F[it->second][it->first])
		{
			ans = *it;
			min = F[it->second][it->first];
		}
	}
	return ans;
}
void getUnclosedNeighbours(cell current,tilemap& t_map,std::set<cell >& closed,std::set<cell >& neighbours)
{
	int height = 0;
	int width  = 0;

	int x = current.first;
	int y = current.second;

	height = t_map.size();
	width  = t_map[0].size();
	neighbours.clear();
	
	switch(t_map[y][x])
	{
		case model::VERTICAL:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x,y+1));
			break;
		}
		case model::HORIZONTAL:{
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			break;
		}
		case model::LEFT_TOP_CORNER:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			break;
		}
		case model::RIGHT_TOP_CORNER:{
			neighbours.insert(cell(x-1,y));
			neighbours.insert(cell(x,y+1));
			break;
		}
		case model::LEFT_BOTTOM_CORNER:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			break;
		}
		case model::RIGHT_BOTTOM_CORNER:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x-1,y));
			break;
		}
		case model::LEFT_HEADED_T:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x-1,y));
			break;
		}
		case model::RIGHT_HEADED_T:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			break;
		}
		case model::TOP_HEADED_T:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			break;
		}
		case model::BOTTOM_HEADED_T:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			break;
		}	
		case model::CROSSROADS:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			break;
		}	
		default:{
			break;
		}	
	}

	for(auto it = neighbours.begin();it != neighbours.end();it++)
	{
		if( it->first >= width || it->first < 0){ neighbours.erase(*it);continue; }
		if( it->second >= height || it->second < 0){ neighbours.erase(*it);continue; }
		if(t_map[it->second][it->first] == model::_UNKNOWN_TILE_TYPE_){ neighbours.erase(*it); }
		if(t_map[it->second][it->first] == model::EMPTY){ neighbours.erase(*it); }
		if(closed.count(*it) > 0){ neighbours.erase(*it); }
	}

	return;
}
