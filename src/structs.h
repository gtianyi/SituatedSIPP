#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
using namespace boost::multi_index;

struct conflict
{
    int agent1;
    int agent2;
    int sec1;
    int sec2;
    double i;
    double j;
    double g;
};


struct Agent
{
    std::string id;
    int id_num;
    int start_i;
    int start_j;
    double start_heading;
    int goal_i;
    int goal_j;
    double goal_heading;
    double size;
    double rspeed;
    double mspeed;
    Agent(){ start_i = -1; start_j = -1; goal_i = -1; goal_j = -1; id_num = -1;
             size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; rspeed = CN_DEFAULT_RSPEED;
             start_heading = CN_DEFAULT_SHEADING; goal_heading = CN_DEFAULT_GHEADING; }
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct SafeInterval
{
    double begin;
    double end;
    int id;
    SafeInterval(double begin_=0, double end_=CN_INFINITY, int id_=0):begin(begin_), end(end_), id(id_) {}
    bool operator== (const SafeInterval& other) const{
      return (begin == other.begin) && (end == other.end);
    }
};

struct Node
{
    Node(int _i=-1, int _j=-1, int h_id=0, double _g=-1, double _F=-1):i(_i),j(_j),g(_g),F(_F),Parent(nullptr), heading_id(h_id){optimal = false;}
    ~Node(){ Parent = nullptr; }
    int     i, j;
    int     open_id;
    int     close_id;
    double  size;
    double  g;
    double  F;
    Node*   Parent;
    double  heading;
    int     heading_id;
    bool    optimal;
    int     interval_id;
    SafeInterval interval;
    bool operator< (const Node& other) const
    {
        if(fabs(this->F - other.F) < CN_EPSILON) //breaking-ties
            return this->g > other.g; //g-max
        else
            return this->F < other.F;
    }
    bool operator== (const Node& other) const{
        return (i == other.i) &&
               (j == other.j) &&
               (Parent == other.Parent) &&
               (interval == other.interval);
    }
    std::size_t hash_value(Node const& n){
        std::size_t seed = 0;
        boost::hash_combine(seed, n.i);
        boost::hash_combine(seed, n.j);
        boos::hash_combine(seed, n.parent);
        boost::hash_combine(seed, n.interval);
        return seed;
    }
};

typedef multi_index_container<
        Node,
        indexed_by<
                    //ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, F)>,
                    ordered_non_unique<identity<Node>>,
                    hashed_non_unique<composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node, int, i), BOOST_MULTI_INDEX_MEMBER(Node, int, j),BOOST_MULTI_INDEX_MEMBER(Node, int, interval_id)>>,
                    hashed_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, open_id)>
        >
> OPEN_container;

struct Focal_Elem
{
    int open_id;
    int leaps;
    double g;
    double F;
    Focal_Elem(const Node& node, int leaps_ = 0)
    {
        open_id = node.open_id;
        g = node.g;
        F = node.F;
        leaps = leaps_;
    }
    bool operator<(const Focal_Elem& other) const
    {
        if(this->leaps == other.leaps)
        {
            if(fabs(this->F - other.F) < CN_EPSILON) //breaking-ties
                return this->g > other.g; //g-max
            else
                return this->F < other.F;
        }
        else
            return this->leaps < other.leaps;
    }
};

typedef multi_index_container<
        Focal_Elem,
        indexed_by<
            ordered_non_unique<identity<Focal_Elem>>,
            hashed_unique<BOOST_MULTI_INDEX_MEMBER(Focal_Elem, int, open_id)>
        >
> Focal_container;

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    obstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};

struct section
{
    section(int _i1=-1, int _j1=-1, int _i2=-1, int _j2=-1, double _g1=-1, double _g2=-1)
        :i1(_i1), j1(_j1), i2(_i2), j2(_j2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):i1(a.i), j1(a.j), i2(b.i), j2(b.j), g1(a.g), g2(b.g){}
    int i1;
    int j1;
    int i2;
    int j2;
    double size;
    double g1;
    double g2;//is needed for goal and wait actions
    double mspeed;
    bool operator == (const section &comp) const {return (i1 == comp.i1 && j1 == comp.j1 && g1 == comp.g1);}

};

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};
#endif
