#ifndef EGRAPH_PLANNER_STATE
#define EGRAPH_PLANNER_STATE

#include<queue>

class LazyAEGListElement;

enum EdgeType{NONE, NORMAL, SNAP, DIRECT_SHORTCUT, GRADIENT_SHORTCUT, SNAP_DIRECT_SHORTCUT, SNAP_GRADIENT_SHORTCUT};

class LazyAEGState: public AbstractSearchState{
  public:
    int id;
    unsigned int v;
    unsigned int g;
    int h;
    short unsigned int iteration_closed;
    short unsigned int replan_number;
    LazyAEGState* best_parent;
    LazyAEGState* expanded_best_parent;
    EdgeType best_edge_type;
    EdgeType expanded_best_edge_type;
    int snap_midpoint;
    int expanded_snap_midpoint;
    bool in_incons;
    std::priority_queue<LazyAEGListElement> lazyList;
    bool isTrueCost;
};

class LazyAEGListElement{
  public:
    LazyAEGListElement(LazyAEGState* p, int ec, bool itc, EdgeType et, int snap_mp){
      parent = p;
      edgeCost = ec;
      isTrueCost = itc;
      edgeType = et;
      snap_midpoint = snap_mp;
    }
    bool operator< (const LazyAEGListElement& other) const{
      return (parent->v + edgeCost > other.parent->v + other.edgeCost);
    }
    LazyAEGState* parent;
    int edgeCost;
    bool isTrueCost;
    EdgeType edgeType;
    int snap_midpoint;
};

#endif
