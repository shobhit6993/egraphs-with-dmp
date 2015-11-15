#ifndef EGRAPH_HEURISTIC_H
#define EGRAPH_HEURISTIC_H

#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<vector>

//templated on the type of the heuristic coordinate
//for example:
//vector<double> for euclidean distance heuristic
//vector<int> for a grid search heuristic (indicating the indices of the cell)
template <typename HeuristicType>
class EGraphHeuristic{

  public:

    //set the goal "heuristic coordinate"
    //between this function and getHeuristic, the E-Graph heuristic computation is done.
    //setGoal will be called once at the start of the planning episode so it can
    //be used for initial computations, whereas getHeuristic will be called many times
    //during planning
    virtual void setGoal(const HeuristicType& goal) = 0;

    //get a heuristic value (int) for a state with this "heuristic coordinate"
    //between this function and setGoal, the E-Graph heuristic computation is done.
    //setGoal will be called once at the start of the planning episode so it can
    //be used for initial computations, whereas getHeuristic will be called many times
    //during planning
    virtual int getHeuristic(const HeuristicType& coord) = 0;

    //This function is used in conjunction with "snap motions" from the EGraphable class. 
    //This function detects if a state has the same "heuristic coordinate" as any states in the
    //E-Graph. If it does, then our heuristic can't provide any more guidance to get us to
    //connect to these states (this occurs when the heuristic coordinate is a down-projection
    //of the state coordinate). The EGraphable class can then use snap motions to try
    //to generate motions to directly connect to these states.
    virtual void getEGraphVerticesWithSameHeuristic(const HeuristicType& coord, std::vector<EGraph::EGraphVertex*>& vertices) = 0;

    //This will run on initialization or any time the E-Graph changes 
    //(a new path has been added or some edges are now invalid). 
    //It is often used to perform a computation using the E-Graph edges to provide
    //quick look-ups during search (like pre-computing the results used by getEGraphVerticesWithSameHeuristic)
    virtual void runPrecomputations(){};

    //Direct shortcuts are used by the planner to quickly jump toward the goal when it has 
    //found a path to a component on the E-Graph. This function takes the component and 
    //returns the state on the component that is "closest" to the goal (according to a 
    //heuristic).
    //Typically, there are very few components. It makes sense to cache the result of this function
    //so that if the planner calls it again with the same component id, it is fast to evaluate.
    virtual void getDirectShortcut(int component, std::vector<EGraph::EGraphVertex*>& shortcuts){};
    //This function give you an opportunity to clear any caches or other computations regarding
    //the shortcut computation between planning episodes.
    virtual void resetShortcuts() = 0;

    void initialize(EGraph* eg){
      eg_ = eg;
      runPrecomputations();
    };

    //sets the epsilon E variable. This is used to penalize distance traveled off of the E-Graph
    //in heuristic space (which the heuristic is computed in)
    virtual void setEpsE(double e){
      epsE_ = e;
    };

  protected:
    EGraph* eg_;
    double epsE_;
};

#endif
