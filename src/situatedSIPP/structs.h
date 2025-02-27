#pragma once
#include <boost/functional/hash.hpp>
#include "../structs.h"
#include "../searchresult.h"
#include "../config.h"
#include <unordered_map>
#include <boost/timer/timer.hpp>
#define PLACES 10
#define NANO 0.000000001

class RTTimer{
  private:
    boost::timer::cpu_timer expansion_timer;
    boost::timer::cpu_timer learning_timer;
    boost::timer::cpu_timer decision_timer;
    boost::timer::cpu_timer si_timer;
  public:
    RTTimer(){
      expansion_timer.stop();
      learning_timer.stop();
      decision_timer.stop();
      si_timer.stop();
    }
    void inline resume_expansion(){
      expansion_timer.resume();
    }
    void inline stop_expansion(){
      expansion_timer.stop();
    }
    void inline resume_learning(){
      learning_timer.resume();
    }
    void inline stop_learning(){
      learning_timer.stop();
    }
    void inline resume_decision(){
      decision_timer.resume();
    }
    void inline stop_decision(){
      decision_timer.stop();
    }
    void inline resume_si(){
      si_timer.resume();
    }
    void inline stop_si(){
      si_timer.stop();
    }

    double elapsed_s()  const{
      //cpu time elapsed in seconds
      double total_time(0);
      DEBUG_MSG_NO_LINE_BREAK_RED("ELAPSED LONG: ");
      DEBUG_MSG_RED(expansion_timer.elapsed().user);
      total_time += NANO * static_cast<double>(expansion_timer.elapsed().user);
      total_time += NANO * static_cast<double>(learning_timer.elapsed().user);
      total_time += NANO * static_cast<double>(decision_timer.elapsed().user);
      return total_time;
    }

    std::string elapsed_time_csv() const{
      std::string formatted_string_to_output = "part,wall,user,system,cpu\nexpansion,";
      formatted_string_to_output.append(expansion_timer.format(PLACES, "%w,%u,%s,%t"));
      formatted_string_to_output.append("\nlearning");
      formatted_string_to_output.append(learning_timer.format(PLACES, "%w,%u,%s,%t"));
      formatted_string_to_output.append("\ndecision");
      formatted_string_to_output.append(decision_timer.format(PLACES, "%w,%u,%s,%t"));
      formatted_string_to_output.append("\nsafeIntervals");
      formatted_string_to_output.append(si_timer.format(PLACES, "%w,%u,%s,%t"));
      return formatted_string_to_output;
    } 
};

class RTNode{
private:
  static std::unordered_map<std::pair<int, int>, double, boost::hash<std::pair<int, int>>> _static_h;
  static std::unordered_map<RTNode, double, boost::hash<RTNode>> _dynamic_h;
  static int dynmode; // 0 location, parent, g; 1 location, interval end
  static std::string expansionOrderStr;

  bool compareNodesF(const RTNode& n1,
                const RTNode& n2) const {
            // Tie break on g-value
            if (n1.F() == n2.F()) {
                return n1.g() > n2.g();
            }
            return n1.F() < n2.F();
  }

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

  RTNode(int _i=-1, int _j=-1, double initial_static_g = 0.0, double initial_dynamic_g = 0.0, int h_id=0):i(_i),j(_j),s_g(initial_static_g),d_g(initial_dynamic_g),Parent(nullptr),heading_id(h_id){
      optimal = false;
  }
  ~RTNode(){ 
      Parent = nullptr; 
  }
 
  //[[depreciated]] double F;
  //[[depreciated]] double g;
  inline double F() const{
      return g() + h();
  }
  Node toNode() const{
    return Node(i, j, heading_id, g(), F());
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
      if(expansionOrderStr == "astar"){
        return compareNodesF(*this, rhs);
      }
      else{
          std::cerr << "unknown expansion order! " << expansionOrderStr << "\n";
          std::cerr << "Must set RTNode expansion order whenever instantiating a new node!\n";
          exit(0);
      }
    /*if(fabs(this->F() - rhs.F()) < CN_EPSILON){ //breaking-ties*/
            //return this->g() > rhs.g(); //g-max
          //}
    //else{
        //return this->F() < rhs.F();
    /*}*/
  }

  bool operator==(const RTNode& other) const{
        return (i == other.i) &&
               (j == other.j) &&
               (Parent == other.Parent) &&
               (s_g == other.static_g()) &&
               (d_g == other.dynamic_g());
  }

  void static set_dynmode(int dm){
    RTNode::dynmode = dm;
  }

  void static set_expansion_order(const std::string& expansionAlgorithmStr){
    RTNode::expansionOrderStr = expansionAlgorithmStr;
  }

  std::size_t hash() const{
      std::size_t seed = 0;
      if (RTNode::dynmode == 0){
        boost::hash_combine(seed, i);
        boost::hash_combine(seed, j);
        boost::hash_combine(seed, Parent);
        boost::hash_combine(seed, s_g + d_g);
      }
      else if (RTNode::dynmode == 1){
        boost::hash_combine(seed, i);
        boost::hash_combine(seed, j);
        boost::hash_combine(seed, interval.begin);
      }
      return seed;
  }
  void debug() const{
        DEBUG_MSG_NO_LINE_BREAK_RED(i);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(j);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(static_g());
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(dynamic_g());
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(static_h());
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_RED(dynamic_h());
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


inline std::list<Node> rtn2nl(const std::list<RTNode>& cont){
    std::list<Node> outc;
    for (const RTNode& rtn: cont){
      outc.push_back(rtn.toNode());
    }
    return outc;
}

inline std::vector<Node> rtn2nv(const std::vector<RTNode>& cont){
    std::vector<Node> outc;
    for (const RTNode& rtn: cont){
      outc.push_back(rtn.toNode());
    }
    return outc;
}

struct RTResultPathInfo:  public ResultPathInfo{
    std::list<RTNode> path;
    std::vector<RTResultPathInfo> iterationPath;
    std::vector<RTNode> sections;
    std::list<RTNode> reexpanded_list;
    RTResultPathInfo()
    {
      ResultPathInfo();
      path.clear();
      sections.clear();
    }
    ResultPathInfo toRPI(){ // make this return instead of mutate
      ResultPathInfo rpi;
      rpi.pathfound = pathfound;
      rpi.pathlength = pathlength;
      rpi.runtime = runtime;
      rpi.expanded = expanded;
      rpi.generated = generated;
      rpi.reopened = reopened;
      rpi.reexpanded = reexpanded;
      rpi.path = rtn2nl(path);
      for (RTResultPathInfo rtrpi: iterationPath){
        rpi.iterationPath.push_back(rtrpi.toRPI());
      }
      rpi.sections = rtn2nv(sections);
      rpi.reexpanded_list = rtn2nl(reexpanded_list);
      return rpi;
    }

};

struct RTSearchResult
{
    bool pathfound;
    std::string agentFate; // survived, died, trapped, timed out
    double makespan;
    double flowtime;
    double runtime;
    unsigned int agents;
    int agentsSolved;
    int tries;
    unsigned long expansions;
    std::string timingInformation;
    std::vector<RTResultPathInfo> pathInfo;


    RTSearchResult() : pathInfo(1)
    {
        pathfound = false;
        agentFate = "survived";
        runtime = 0;
        flowtime = 0;
        makespan = 0;
        agents = 0;
        expansions = 0;
        timingInformation = "";
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
