#pragma once
#include <boost/functional/hash.hpp>
#include "../structs.h"
#include <unordered_map>

class RTNode{
private:
  static std::unordered_map<std::pair<int, int>, double, boost::hash<std::pair<int, int>>> _static_h;
  static std::unordered_map<RTNode, double, boost::hash<RTNode>> _dynamic_h;
public:
  int     i, j;
  double  size;
  double s_g, d_g;  // node can have a
  RTNode*   Parent;
  double  heading;
  int     heading_id;
  bool    optimal;
  int     interval_id;
  SafeInterval interval;

  RTNode(int _i=-1, int _j=-1, double initial_static_g = 0.0,  double initial_dynamic_g = 0.0, int h_id=0):i(_i),j(_j),s_g(initial_static_g),d_g(initial_dynamic_g),Parent(nullptr),heading_id(h_id){optimal = false;}
  ~RTNode(){ Parent = nullptr; }
  //[[depreciated]] double F;
  //[[depreciated]] double g;
  inline double F() const{
      return g() + h();
  }
  inline double g() const{
    return static_g() + dynamic_g();
  }
  inline double h() const{
    return static_h() + dynamic_h();
  }
  std::pair<int, int> static_key()const{
     return std::pair<int, int>(i, j);
  }
  double static_h() const{
    return _static_h[static_key()];
  }
  double dynamic_h() const{
     return _dynamic_h[*this];
  }
  double static_g() const{
    return s_g;
  }
  double dynamic_g() const{
      return d_g;
  }
  void set_static_h(double val){
    _static_h[static_key()] = val;
  }
  void set_dynamic_h(double val){
    _dynamic_h[*this] = val;
  }
  void set_static_g(double val){
    s_g = val;
  }
  void set_dynamic_g(double val){
    d_g = val;
  }

  void set_zero(){
    this->set_static_g(0.0);
    this->set_dynamic_g(0.0);
    this->set_dynamic_h(0.0);
    this->set_static_h(0.0);
  }
  void set_inf(){
    double inf = std::numeric_limits<double>::infinity();
    this->set_static_g(inf);
    this->set_dynamic_g(0.0);
    this->set_dynamic_h(0.0);
    this->set_static_h(inf);
  }

  bool operator< (const RTNode& rhs) const {
    if(fabs(this->F() - rhs.F()) < CN_EPSILON){ //breaking-ties
            return this->g() > rhs.g(); //g-max
          }
    else{
        return this->F() < rhs.F();
    }
  }

  bool operator==(const RTNode& other) const{
        return (i == other.i) &&
               (j == other.j) &&
               (Parent == other.Parent) &&
               (s_g == other.static_g()) &&
               (d_g == other.dynamic_g());
  }
  std::size_t hash() const{
      std::size_t seed = 0;
      boost::hash_combine(seed, i);
      boost::hash_combine(seed, j);
      boost::hash_combine(seed, Parent);
      boost::hash_combine(seed, s_g);
      boost::hash_combine(seed, d_g);
      return seed;
  }
};



#ifdef BOOST_NO_ARGUMENT_DEPENDENT_LOOKUP
namespace boost{
  inline std::size_t hash_value(const RTNode& n)
    {
        return n.hash();
    }
}
#else
inline std::size_t hash_value(const RTNode& n)
  {
      return n.hash();
  }
#endif



typedef multi_index_container<
        RTNode,
        indexed_by<
                    //ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, F)>,
                    ordered_non_unique<identity<RTNode>>,
                    hashed_non_unique<composite_key<RTNode, BOOST_MULTI_INDEX_MEMBER(RTNode, int, i), BOOST_MULTI_INDEX_MEMBER(RTNode, int, j),BOOST_MULTI_INDEX_MEMBER(RTNode, int, interval_id)>>,
                    hashed_unique<identity<RTNode>>
        >
> RTOPEN_container;


struct RTResultPathInfo
{
    bool pathfound;
    double pathlength;
    double runtime;
    std::list<RTNode> path;
    std::vector<RTResultPathInfo> iterationPath;
    std::vector<RTNode> sections;
    int expanded;
    int generated;
    int reopened;
    int reexpanded;
    std::list<RTNode> reexpanded_list;

    RTResultPathInfo()
    {
        runtime = 0;
        pathfound = false;
        pathlength = 0;
        path.clear();
        sections.clear();
    }
};

struct RTSearchResult
{
    bool pathfound;
    double makespan;
    double flowtime;
    double runtime;
    unsigned int agents;
    int agentsSolved;
    int tries;
    std::vector<RTResultPathInfo> pathInfo;


    RTSearchResult() : pathInfo(1)
    {
        pathfound = false;
        runtime = 0;
        flowtime = 0;
        makespan = 0;
        agents = 0;
    }

    ~RTSearchResult()
    {
        pathInfo.clear();
    }

};

struct RTobstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<RTNode> sections;
    RTobstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};
