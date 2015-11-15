/*
 * Copyright (c) 2013, Mike Phillips and Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _EGRAPH_PLANNER_H_
#define _EGRAPH_PLANNER_H_

#include <sbpl/headers.h>
#include <queue>
#include <egraphs/egraphManager.h>
#include <egraphs/planner_state.h>

//class LazyAEGListElement;

class EGraphReplanParams : public ReplanParams{
  public:
    EGraphReplanParams(double time):ReplanParams(time) {
      epsE = 10.0;
      final_epsE = 1.0;
      dec_epsE = 1.0;
      feedback_path = true;
      use_egraph = true;
      use_lazy_validation = true;
    };
    double epsE;
    double final_epsE;
    double dec_epsE;
    bool feedback_path;
    bool use_egraph;
    bool update_stats;
    bool use_lazy_validation;
    bool validate_during_planning;
};

template <typename HeuristicType>
class LazyAEGPlanner : public SBPLPlanner{
    typedef EGraphManager<HeuristicType>* EGraphManagerPtr;

    public:
        void interrupt();

        virtual int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V){
            printf("Not supported. Use ReplanParams");
            return -1;
        };
        virtual int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost){
            printf("Not supported. Use ReplanParams");
            return -1;
        };

        virtual int replan(int start, vector<int>* solution_stateIDs_V, 
                           EGraphReplanParams params, int* solcost);
        virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params);
        virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params, int* solcost);

        virtual int set_goal(int goal_stateID){ROS_WARN("set_goal is not used. we assume the goal conditions have been set in the environment and use EGraphable::isGoal");return 1;};
        virtual int set_goal();
        virtual int set_start(int start_stateID);

        virtual void costs_changed(StateChangeQuery const & stateChange){return;};
        virtual void costs_changed(){return;};

        virtual int force_planning_from_scratch(){return 1;};
        virtual int force_planning_from_scratch_and_free_memory(){return 1;};

        virtual int set_search_mode(bool bSearchUntilFirstSolution){
            printf("Not supported. Use ReplanParams");
            return -1;
        };

        virtual void set_initialsolution_eps(double initialsolution_eps){
            printf("Not supported. Use ReplanParams");
        };

        LazyAEGPlanner(DiscreteSpaceInformation* environment, bool bforwardsearch, 
                       EGraphManagerPtr egraph_mgr);
        ~LazyAEGPlanner(){};

        map<string,double> getStats(){return stat_map_;};
        virtual void get_search_stats(vector<PlannerStats>* s);
        void feedback_last_path();
        void setLazyValidation(bool b){ params.use_lazy_validation = b; };
        void setValidateDuringPlanning(bool b){ params.validate_during_planning = b; };

    protected:
        //data structures (open and incons lists)
        CHeap heap;
        vector<LazyAEGState*> incons;
        vector<LazyAEGState*> states;

        EGraphReplanParams params;
        EGraphManagerPtr egraph_mgr_;

        bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
        LazyAEGState goal_state;
        LazyAEGState* start_state;
        //int goal_state_id;
        int start_state_id;

        //search member variables
        double eps;
        double eps_satisfied;
        int search_expands;
        clock_t TimeStarted;
        short unsigned int search_iteration;
        short unsigned int replan_number;
        bool use_repair_time;

        //stats
        map<string,double> stat_map_;
        vector<PlannerStats> stats;
        unsigned int totalExpands;
        double totalPlanTime;
        double reconstructTime;
        double feedbackPathTime;
        double heuristicSetGoalTime;
        clock_t succsClock;
        clock_t shortcutClock;
        clock_t snapClock;
        clock_t heuristicClock;
        double percentFromShortcuts;

        int evaluated_snaps;

        bool interruptFlag;

        bool reconstructSuccs(LazyAEGState* state, LazyAEGState*& next_state, 
                              vector<int>* wholePathIds, vector<int>* costs);

        virtual LazyAEGState* GetState(int id);
        virtual void ExpandState(LazyAEGState* parent);
        virtual void EvaluateState(LazyAEGState* parent);
        void getNextLazyElement(LazyAEGState* state);
        void insertLazyList(LazyAEGState* state, LazyAEGState* parent, int edgeCost, bool isTrueCost, EdgeType edgeType, int snap_midpoint);
        void putStateInHeap(LazyAEGState* state);
        void updateGoal(LazyAEGState* state);

        virtual int ImprovePath();

        virtual vector<int> GetSearchPath(int& solcost);

        virtual bool outOfTime();
        virtual void initializeSearch();
        virtual void prepareNextSearchIteration();
        virtual bool Search(vector<int>& pathIds, int & PathCost);

};

#include<egraphs/../../src/egraph_planner.cpp>

#endif
