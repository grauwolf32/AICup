#include <utility>
#include <vector>
#include <iostream>
#include <set>
#include <map>
#include <math.h>
#include "TileType.h"
#include "as.h"

const int infinite = 0x7FFFFFFF;
const int dist_coeff = 10;

int findPathA(cell start,cell end,path& best_path,tilemap& t_map, double (*h)(cell x,cell y) )
{
	int width = 0;
	int height = 0;
	int is_ok = 0;
	cell current;
	
	width = t_map[0].size();
	height = t_map.size();
	
	std::cout << "height :"<<height<<std::endl;
	std::cout << "width :"<<width<<std::endl;

	costmap G(height,std::vector<double>(width, infinite));
	costmap F(height,std::vector<double>(width, infinite));

	G[start.first][start.second] = 0;
	F[start.first][start.second] = G[start.first][start.second] + dist_coeff*h(start,end);

	std::set<cell > wawe;
	std::set<cell > closed;
	std::map<cell,cell > from;
	std::set<cell > unclosedNeighbours; 
	
	wawe.insert(start);

	while(!wawe.empty())
	{
		current = findMinF(wawe,F);
		std::cout << "current : "<< current.first << " " << current.second << std::endl;
		if(current == end){is_ok = 1;break;}
		wawe.erase(current);
		closed.insert(current);
		getUnclosedNeighbours(current,t_map,closed,unclosedNeighbours);
		for(auto it = unclosedNeighbours.begin();it != unclosedNeighbours.end();it++)
		{
			double temp_G = G[current.first][current.second] + transition_cost(from[current],current,*it);
			if(wawe.count(*it) == 0 || temp_G < G[it->first][it->second])
			{
				from[*it] = current;
				G[it->first][it->second] = temp_G;
				F[it->first][it->second] = G[it->first][it->second] + dist_coeff*h(*it,end);
				if(wawe.count(*it) == 0) wawe.insert(*it);
			}
			std::cout << "wawe :"<< std::endl;
			for(auto it = wawe.begin();it != wawe.end();it++)
				std::cout << it->first << " " << it->second << std::endl;
			std::cout << std::endl;
		}
	}
	std::cout <<"F: "<< std::endl;
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
			std::cout << F[i][j] << " ";
		std::cout << std::endl;
	}
	std::cout << std::endl;

	std::cout <<"G: "<< std::endl;
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
			std::cout << G[i][j] << " ";
		std::cout << std::endl;
	}
	std::cout << std::endl;

	if(!is_ok) return -1;
	
	best_path.clear();

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
		if(min > F[it->first][it->second])
		{
			ans = *it;
			min = F[it->first][it->second];
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
	
	switch(t_map[x][y])
	{
		case model::VERTICAL:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x,y-1));
			std::cout << "VERTICAL "  << std::endl;
			break;
		}
		case model::HORIZONTAL:{
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			std::cout << "HORIZONTAL" << std::endl;
			break;
		}
		case model::LEFT_TOP_CORNER:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			std::cout << "LEFT_TOP_CORNER" << std::endl;
			break;
		}
		case model::RIGHT_TOP_CORNER:{
			neighbours.insert(cell(x-1,y));
			neighbours.insert(cell(x,y-1));
			std::cout << "RIGHT_TOP_CORNER" << std::endl;
			break;
		}
		case model::LEFT_BOTTOM_CORNER:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			std::cout << "LEFT_BOTTOM_CORNER" << std::endl;
			break;
		}
		case model::RIGHT_BOTTOM_CORNER:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x-1,y));
			std::cout << "RIGHT_BOTTOM_CORNER" << std::endl;
			break;
		}
		case model::LEFT_HEADED_T:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x-1,y));
			std::cout << "LEFT_HEADED_T" << std::endl;
			break;
		}
		case model::RIGHT_HEADED_T:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			std::cout << "RIGHT_HEADED_T"<< std::endl;
			break;
		}
		case model::TOP_HEADED_T:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			std::cout << "TOP_HEADED_T" << std::endl;
			break;
		}
		case model::BOTTOM_HEADED_T:{
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			std::cout << "BOTTOM_HEADED_T" << std::endl;
			break;
		}	
		case model::CROSSROADS:{
			neighbours.insert(cell(x,y+1));
			neighbours.insert(cell(x,y-1));
			neighbours.insert(cell(x+1,y));
			neighbours.insert(cell(x-1,y));
			std::cout << "CROSSROADS" << std::endl;
			break;
		}	
		default:{
			std::cout << "DEFAULT" << std::endl;
			break;
		}	
	}
	for(auto it = neighbours.begin();it != neighbours.end();it++)
	{
		if( it->first > width || it->first < 0){neighbours.erase(*it); std::cout << it->first << " "<< it->second << " " << "x coordinate out of range!" <<std::endl;}
		if( it->second > height || it->second < 0){neighbours.erase(*it); std::cout << it->first << " "<< it->second << " " << "y coordinate out of range!" <<std::endl;}
		if(t_map[it->first][it->second] == model::_UNKNOWN_TILE_TYPE_){neighbours.erase(*it);std::cout << "Unknown tile type" << std::endl;}
		if(t_map[it->first][it->second] == model::EMPTY)neighbours.erase(*it);{std::cout << "Empty tile"<<std::endl;}
	}
	std::cout << "unclosed neighbours: "<< std::endl;
	for(auto it = neighbours.begin();it != neighbours.end();it++)
	{
		std::cout << it->first << " " << it->second << std::endl;
	}
	std::cout << std::endl;
	return;
}
