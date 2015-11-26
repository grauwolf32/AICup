#include <vector>
#include <string>
#include <cmath>
#include <set>
#include <map>

typedef std::vector<std::pair<int,int> > path;
typedef std::pair<int,int> cell;
typedef std::vector<std::vector<model::TileType> > tilemap;
typedef std::vector<std::vector<double> > costmap;
typedef std::vector<quad> quadpath;
typedef std::vector<interface> sectors;

namespace addons
{
	const double epsilon = 10e-5;
	bool is_equal(double x, double y);

	enum TurnTypes{
		_UNKNOWN_TURN_ = -1,
		 STRAIGHT =  0,
     	 UPRIGHT  =  1,
		 UPLEFT   =  2,
		 DOWNLEFT =  3,
		 DOWNRIGHT = 4,
		 LEFTUP  = 5,
		 LEFTDOWN = 6,
		 RIGHTUP = 7,
		 RIGHTDOWN = 8,
		_TURN_COUNT_ = 9
		};

	bool is_equal(double x, double y)
	{
		if(x > y - epsilon && x < y + epsilon) return true;
		return false;
	}
}

class point2d
{
	public:
		point2d();
	    ~point2d();

		point2d(double x,double y);

		point2d operator+(const point2d& rhs);
		point2d operator-(const point2d& rhs);
		point2d operator*(const double alpha);
		bool    operator==(const point2d& rhs);

		double x;
		double y;	
};

class vector
{
	public:
		vector2d();
	    ~vector2d();
		vector2d(point2d);
		vector2d(point2d, point2d);
		vector2d(double x,double y);
		
		vector2d operator+(const vector2d& rhs);
		vector2d operator-(const vector2d& rhs);
		vector2d operator*(const double alpha);
		bool     operator==(const vector2d& rhs);
		
		vector2d normalize();
		vector2d normal();
		vector2d rotate(const double alpha);

		double length();

		point2d head;
};

class line
{
	public:
		line();
	    ~line();	
		line(point2d p1,point2d p2);
		line(point2d p1, vector2d v1);
		line(double a, double b, double c);
	
	int a;
	int b;
	int c;
	
	point2d  point;
	vector2d direction;
	
};

class segment
{
	public:
		segment();
		segment(point2d v1,point2d v2);
		segment(vector2d v1, vector2d v2);
	    ~segment();
	
		point2d v1;
		point2d v2;
};

class interface:public segment;

class quad
{
	public:
		quad();
		quad(point2d v1,point2d v2,point2d v2,point2d v4);	
		quad(vector2d v1, vector2d v2, vector2d v3, vector2d v4);	

		point2d v1;	
		point2d v2;
		point2d v3;
		point2d v4;
}

point2d findLinesIntersection(const line l1,const line l2);
point2d findSegmentsIntersection(const segment s1,const segment s2);

void correctPaddingX(point2d& p1,point2d& p2,double padding);
void correctPaddingY(point2d& p1,point2d& p2,double padding);

void  quadsFromPath(quadpath& quadPath, path& best_path);
void  getPathInterfaces(sectors& interfaces,quadpath& quadPath path& best_path);

TurnType getTurnType(cell c_last,cell c_current cell c_next);
Direction getDirection(cell c1,cell c2);

double getAngle(vector2d v1, vector2d v2);
double getAngle(line l1, line l2);

double crossproduct(vector2d v1, vector2d v2);
double dotproduct(vector2d v1, vector2d v2);

bool  isParallel(vector2d v1, vector2d v2);
bool  isParallelline l1, line l2);

quad  getQuad(cell c,double tile_size);


quad  getQuad(cell c,double tile_size) /*From upper left vertex to others, moving clockwise*/
{
	point2d v1(c.first*tile_size,c.second*tile_size);
	point2d v2((c.first+1)*tile_size,c.second*tile_size);
	point2d v3((c.first +1)*tile_size,(c.second+1)*tile_size);
	point2d v4(c.first*tile_size,(c.second+1)*tile_size);
	return quad(v1,v2,v3,v4);
}

void  quadsFromPath(sectors& quadPath, path& best_path,double tile_size)
{
	quadPath.clear();
	quad  temp_block;
	int n = best_path.size();
	if( n <= 0) { return; }

	for(int i = 0;i < n;i++)
	{
		temp_block = getQuad(path[i],tile_size);
		quadPath.push_back(temp_block);
	}
	
	return;	
}

Direction getDirection(cell c1,cell c2)
{
	if(c1.first == c2.first)
	{
		if(c1.second = c2.second -1){return model::UP;}
		if(c1.second = c2.second +1){return model:DOWN;}
	}
	if(c1.second == c2.second)
	{
		if(c1.first = c2.first+1){return model::LEFT;}
		if(c1.first = c2.first-1){return model::RIGHT;}
	}
	return _UNKNOWN_DIRECTION_;
} 

void correctPaddingX(point2d& p1, point2d& p2,double padding)
{	
	if(p1.first > p2.first)
	{
		p1.first -= padding;
		p2.first += padding;
		return;
	}
	p1.first += padding;
	p2.first -= padding;
	return;
}

void correctPaddingY(point2d& p1,point2d& p2,double padding)
{
	if(p1.second > p2.second)
	{
		p1.second -= padding;
		p2.second += padding;
		return;
	}
	p1.second -= padding;
	p2.second += padding;
	return;
}

TurnType getTurnType(cell c_last,cell c_current cell c_next)
{
	using namespace addons;
	using namespace model;

	Direction d1 = getDirection(c_last,c_current);
	Direction d2 = getDirection(c_current,c_next);
	
	if(d1 == d2) return STRAIGHT;

	if(d1 == LEFT)
	{
		if(d2 == UP) { return LEFTUP;}
		if(d2 = DOWN){ return LEFTDOWN;}
	}
	if(d1 == RIGHT)
	{
		if(d2 == UP) { return RIGHTUP;}
		if(d2 = DOWN){ return RIGHTDOWN;}
	}

	if(d1 == UP)
	{
		if(d2 == LEFT) { return UPLEFT; }
		if(d2 == RIGHT){ return UPRIGHT; }
	}

	if(d1 == DOWN)
	{
		if(d2 == LEFT) { return DOWNLEFT; }
		if(d2 == RIGHT){ return DOWNRIGHT; }
	}
	return _UNKNOWN_TURN_;
}

void getPathIntefaces(sectors& interfaces, quadpath& quadPath, path& best_path, double padding)
{
	cell prev;
	cell curr;
	cell next;

	TurnType turn;
	Direction dir;

	point2d v1;
	point2d v2;

	interface temp_interface;

	interfaces.clear();
	int n = best_path.size();

	if( n == 1)
	{
		return; /*We must disallow paths with length one or we mustn't ?*/
	}

	int i = 0;	
	curr = best_path[i];
	prev = current;
	next = best_path[i+1];
											// Initialization
	dir = getDirection(prev,curr,next);
	if(dir == UP){
			v1 = quadPath[i].v1;
			v2 = quadPath[i].v2;
			correctPaddingX(v1,v2,padding);
			interfaces.push_back(interface(v1,v2));}
	if(dir == RIGHT){
			v1 = quadPath[i].v2;
			v2 = quadPath[i].v3;
			correctPaddingY(v1,v2,padding);
			interfaces.push_back(interface(v1,v2));}
	if(dir == DOWN){
			v1 = quadPath[i].v3;
			v2 = quadPath[i].v4;
			correctPaddingX(v1,v2,padding);
			interfaces.push_back(interface(v1,v2));}
	if(dir == LEFT){
			v1 = quadPath[i].v4;
			v2 = quadPath[i].v1;
			correctPaddingY(v1,v2,padding);
			interfaces.push_back(interface(v1,v2));}
											// Ordered interfaces enumeration
	for(i = 1; i < n;i++)
	{
		if(i == n-1){ next = best_path[n-1]; }
		else { next = best_path[i+1]; }
		curr = best_path[i];

		turn = getTurnType(prev,curr,next);
		dir  = getDirection(prev,curr);

		switch(Turn)
		{
			case STRAIGHT : break;

			case UPLEFT :
			case DOWNRIGHT :
			case RIGHTDOWN :
			case LEFTUP :{
				v1 = quadPath[i].v1;
				v2 = quadPath[i].v3;
				correctPaddingX(v1,v2,padding);
				correctPaddingY(v1,v2,padding);
				break;
			}

			case UPRIGHT :
			case RIGHTUP :
			case DOWNLEFT :
			case LEFTDOWN :{
				v1 = quadPath[i].v2;
				v2 = quadPath[i].v4;
				correctPaddingX(v1,v2,padding);
				correctPaddingY(v1,v2,padding);
				break;
			}

			default : break; /* TODO May be add some logic here */
		}

		switch(dir)
		{
			case UP:{ 
				v1 = quadPath[i].v1; 
				v2 = quadPath[i].v2;
				interfaces.push_back(interface(v1,v2));	
				break;
			}
			case DOWN:{ 
				v1 = quadPath[i].v3;
				v2 = quadPath[i].v4;
				interfaces.push_back(interface(v1,v2));
				break;
			}
			case LEFT:{ 
				v1 = quadPath[i].v4;
				v2 = quadPath[i].v1;
			i	nterfaces.push_back(interface(v1,v2));
				break
			}
			case RIGHT:{
				v1 = quadPath[i].v2;
				v2 = quadPath[i].v3;
				interfaces.push_back(interface(v1,v2));}
				break;
			}
			default break; /* TODO May be add some logic here */
		}
		prev = current;
	}
	return;
}

point2d::point2d() {}
point2d::~point2d(){}

point2d::point2d(double x_,double y_):x(x_),y(y_){}

point2d point2d::operator+(const point2d rhs)
{
	return point2d(x + rhs.x,y + rhs.y);	
}
point2d point2d::operator+(const point2d rhs)
{
	return point2d(x - rhs.x,y - rhs.y);
}
point2d point2d::operator*(const double alpha)
{
	return point2d(x*alpha,y*alpha);
}
bool point2d::operator==(const point2d& rhs)
{
	return addons::is_equal(x,rhs.x) & addons::is_equal(y,rhs.y);
}

vector2d::vector2d(){}
vector2d::vector2d(point2d v1):head(v1){}
vector2d::vector2d(point2d v1,point2d v2):head(v2-v1){}
vector2d::vector2d(double x,double y):head(point2d(x,y)){}

vector2d vector2d::operator+(const vector2d& rhs)
{
	return vector2d(head + rhs.head);
}
vector2d vector2d::operator-(const vector2d& rhs)
{
	return vector2d(head - rhs.head);
}
vector2d vector2d::operator*(const double alpha)
{
	return vector2d(head * alpha);
}
vector2d vector2d::operator==(const vector2d& rhs)
{
	return head == rhs.head;
}
vector2d vector2d::normalize()
{
	double length = this->length();
	if(addons::is_equal(length,0.0)) return vector2d(0.0,0.0);
	return (*this) * (1.0/length);
}
vector2d vector2d::normal()
{
	if((*this) == vector2d(0.0,0.0))
		return vector2d(0.0,0.0);
	return vector2d(-head.y,head.x);
}
vector2d vector2d::rotate(const double alpha)
{
	if((*this) == vector2d(0.0,0.0)){
		return vector2d(0.0,0.0);
	}

	double x = head.x;
	double y = head.y;
	
	double x_ = x*cos(alpha) - y*sin(alpha);
	double y_ = x*sin(alpha) + y*cos(alpha);

	return vector2d(x_,y_);
}

double  vector2d::length()
{
	return sqrt(x*x + y*y); /*TODO add reference to euclidian norm function */
}

double crossproduct(vector2d v1, vector2d v2)
{
	return (v1.head.x*v2.head.y - v1.head.y*v2.head.x);
}
double dotproduct(vector2d v1, vector2d v2)
{
	return (v1.head.x * v2.head.x + v1.head.y*v2.head.y);
}
double getAngle(vector2d a, vector2d b)
{
	return atan2(crossproduct(a,b),dotproduct(a,b));
}
double getAngle(line a, line b)
{
	return getAngle(a.direction, b.direction);
}

line::line(){}
line::line(point2d a, vector2d v1):point(a),direction(v1)
{
	/*TODO Calculate a, b and c */
}
line::line(point2d a, point2d b)
{	
	return line(a,vector2d(b,a));
}
line::line(double a_, double b_, double c_)a(a_),b(b_),c(c_)
{
	/*TODO Calculate vector and find point */
}

segment::segment(){}
segment::segment(point2d v1_,point2d v2_):v1(v1_),v2(v2_){}
segment::segment(vector2d v1_,vector2d v2_):v1(v1.head),v2(v2.head){}

quad::quad(){}
quad::quad(point2d v1_, point2d v2_, point2d v3_, point2d v4_):v1(v1_),v2(v2_),v3(v3_),v4(v4_):{}
quad(vector2d v1_, vector2d v2_, vector2d v3_, vector2d v4_):v1(v1_.head),v2(v2_.head),v3(v3_.head),v4(v4_.head){}



