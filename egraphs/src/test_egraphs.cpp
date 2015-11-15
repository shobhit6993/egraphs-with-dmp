#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<egraphs/egraph_2d_grid_heuristic.h>
#include<egraphs/egraph_planner.h>
#include<sbpl/headers.h>

class myEnv : public EnvironmentNAVXYTHETALAT, public EGraphable<vector<int> >, public EGraphDiscretize{
    //requires a snap motion function between 2 coordinates. returns true if a transition exists (and then also fills out a cost and state id). 
    //this is used for snap motions as well as reading in demonstrations (so it's important for the environment to attempt to use motiom primitives first and then adaptive motions)
    virtual bool snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
      return false;
    };

    //requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
    //-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)
    virtual bool getCoord(int id, vector<double>& coord){
      return true;
    };

    virtual void getGoalHeuristicCoord(vector<double>& coord){
    };

    virtual bool getGoalCoord(const vector<double>& parent, vector<double>& goal){
      return true;
    };

    virtual bool isGoal(int id){
      return false;
    };

    virtual int getStateID(const vector<double>& coord){
      return 0;
    };

    virtual bool isValidEdge(const vector<double>& coord, const vector<double>& coord2, bool& change_cost, int& cost){
      return true;
    };

    virtual bool isValidVertex(const vector<double>& coord){
      return true;
    };

    void projectToHeuristicSpace(const vector<double>& coord, vector<int>& heur_coord) const{

    };

    void projectGoalToHeuristicSpace(vector<int>& heur_coord) const{

    };

    void discToCont(const vector<int>& d, vector<double>& c){

    };

    void contToDisc(const vector<double>& c, vector<int>& d){

    };

};

int main(int argc, char** argv){
  myEnv env;

  EGraph eg(&env,0,0);
  EGraph2dGridHeuristic heur(env,100,100,1);

  EGraphManager<vector<int> > egraph_mgr(&eg, &env, &heur);
  LazyAEGPlanner<vector<int> > aegplanner(&env,true,&egraph_mgr);

  sleep(2.0);
  return 0;
}
