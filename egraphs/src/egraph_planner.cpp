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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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
#include <egraphs/egraph_planner.h>
#include <algorithm>
#include <numeric>

using namespace std;

template <typename HeuristicType>
LazyAEGPlanner<HeuristicType>::LazyAEGPlanner(DiscreteSpaceInformation* environment, bool bSearchForward,
                               EGraphManagerPtr egraph_mgr) :
  params(0.0), egraph_mgr_(egraph_mgr){ //, goal_state(NULL) {
  //bforwardsearch = bSearchForward;
  if(!bSearchForward)
    ROS_WARN("backward search not supported. setting to run forward search.");
  bforwardsearch = true;
  environment_ = environment;
  replan_number = 0;

  //goal_state_id = -1;
  start_state_id = -1;
  evaluated_snaps = 0;
}

template <typename HeuristicType>
LazyAEGState* LazyAEGPlanner<HeuristicType>::GetState(int id){	
  //if this stateID is out of bounds of our state vector then grow the list
  if(id >= int(states.size())){
    for(int i=states.size(); i<=id; i++)
      states.push_back(NULL);
  }
  //if we have never seen this state then create one
  if(states[id]==NULL){
    states[id] = new LazyAEGState();
    states[id]->id = id;
    states[id]->replan_number = -1;
  }
  //initialize the state if it hasn't been for this call to replan
  LazyAEGState* s = states[id];
  if(s->replan_number != replan_number){
    s->g = INFINITECOST;
    s->v = INFINITECOST;
    s->iteration_closed = -1;
    s->replan_number = replan_number;
    s->best_parent = NULL;
    s->expanded_best_parent = NULL;
    s->best_edge_type = EdgeType::NONE;
    s->expanded_best_edge_type = EdgeType::NONE;
    s->snap_midpoint = -1;
    s->expanded_snap_midpoint = -1;
    s->heapindex = 0;
    s->in_incons = false;
    s->isTrueCost = true;
    //clear the lazy list
    while(!s->lazyList.empty())
      s->lazyList.pop();

    //compute heuristics
    if(bforwardsearch){
      clock_t h_t0 = clock();
      s->h = egraph_mgr_->getHeuristic(s->id);
      clock_t h_t1 = clock();
      heuristicClock += h_t1-h_t0;
    } else {
      printf("backwards search not implemented!");
      assert(false);
      s->h = environment_->GetStartHeuristic(s->id);
    }
  }
  return s;
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::ExpandState(LazyAEGState* parent){
  bool print = false; //parent->id == 285566;
  if(print)
    printf("expand %d\n",parent->id);
  vector<int> children;
  vector<int> costs;
  vector<bool> isTrueCost;

  clock_t getSucc_t0 = clock();
  if(bforwardsearch)
    environment_->GetLazySuccsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  else
    environment_->GetLazyPredsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  clock_t getSucc_t1 = clock();
  succsClock += getSucc_t1-getSucc_t0;

  vector<EdgeType> edgeTypes(children.size(),EdgeType::NORMAL);
  
  vector<int> snap_midpoints(children.size(),-1);
  if (params.use_egraph){
    //egraph_mgr_->clearSnapSuccessorsCache();

    //egraph_mgr_->getSnapSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    clock_t shortcut_t0 = clock();
    egraph_mgr_->getDirectShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);

    snap_midpoints.resize(children.size(),-1);
    egraph_mgr_->getSnapShortcuts(parent->id, &children, &costs, &isTrueCost, &edgeTypes, &snap_midpoints);
    if(edgeTypes.size()>0 && edgeTypes.back()==EdgeType::SNAP_DIRECT_SHORTCUT)
      assert(snap_midpoints.back()>=0);
    clock_t shortcut_t1 = clock();
    shortcutClock += shortcut_t1 - shortcut_t0;

    //egraph_mgr_->getComboSnapShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    // getComboSnapShortcutSuccessors needs the output of getSnapSuccessors, so
    // getSnapSuccessors caches its output inside egraph_mgr_. we must make sure
    // to clean it up before the next iteration
    //egraph_mgr_->clearSnapSuccessorsCache();
  }
  
  // make sure our costs are nonzero
  for (auto& cost : costs){
      assert(cost > 0);
  }
  if (print){
      for (auto& id : children){
          ROS_INFO("%d", id);
      }
  }

  //iterate through children of the parent
  for(int i=0; i<(int)children.size(); i++){
    //printf("  succ %d\n",children[i]);
    LazyAEGState* child = GetState(children[i]);
    insertLazyList(child, parent, costs[i], isTrueCost[i], edgeTypes[i], snap_midpoints[i]);
  } 
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::EvaluateState(LazyAEGState* state){
  bool print = false; //state->id == 285566;
  if(print)
    printf("evaluate %d (from %d)\n",state->id, state->best_parent->id);
  LazyAEGState* parent = state->best_parent;
  EdgeType edgeType = state->best_edge_type;
  int snap_midpoint = state->snap_midpoint;

  getNextLazyElement(state);
  //printf("state_ptr=%p\n",state);
  //printf("state_id=%d\n",state->id);
  //printf("parent_ptr=%p\n",parent);
  //printf("parent_id=%d\n",parent->id);
  
  /* Mike replacing victor's code
  Edge snap_edge(parent->id, state->id);
  int trueCost;
  if (egraph_mgr_->snaps_.find(snap_edge) != egraph_mgr_->snaps_.end()){
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  } else {
    trueCost = environment_->GetTrueCost(parent->id, state->id);
  }
  */

  int trueCost;
  if(edgeType == EdgeType::SNAP)
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  else if(edgeType == EdgeType::NORMAL){
    clock_t getSucc_t0 = clock();
    trueCost = environment_->GetTrueCost(parent->id, state->id);
    clock_t getSucc_t1 = clock();
    succsClock += getSucc_t1-getSucc_t0;
  }
  else if(edgeType == EdgeType::SNAP_DIRECT_SHORTCUT){
    clock_t snap_t0 = clock();
    assert(snap_midpoint >= 0);
    trueCost = egraph_mgr_->getSnapShortcutTrueCost(parent->id, snap_midpoint, state->id);
    clock_t snap_t1 = clock();
    snapClock += snap_t1 - snap_t0;
    //this is technically a snap-shortcut, however, when expanding a state,
    //the bulk of the work is finding the shortcut and the snap is evaluated lazily
    //here, the shortcut was cached from before and the snap is fully evaluated so 
    //this is dominated by the snap time (there are some nuances though)
  }
  else
    assert(false);

  if (print)
      printf("has a true cost of %d\n",trueCost);
  if(trueCost > 0) //if the evaluated true cost is valid (positive), insert it into the lazy list
    insertLazyList(state,parent,trueCost,true,edgeType,snap_midpoint);
}

//this should only be used with EvaluateState since it is assuming state hasn't been expanded yet (only evaluated)
template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::getNextLazyElement(LazyAEGState* state){
  if(state->lazyList.empty()){
    state->g = INFINITECOST;
    state->best_parent = NULL;
    state->best_edge_type = EdgeType::NONE;
    state->snap_midpoint = -1;
    state->isTrueCost = true;
    return;
  }
  LazyAEGListElement elem = state->lazyList.top();
  state->lazyList.pop();
  state->g = elem.parent->v + elem.edgeCost;
  state->best_parent = elem.parent;
  state->best_edge_type = elem.edgeType;
  state->snap_midpoint = elem.snap_midpoint;
  state->isTrueCost = elem.isTrueCost;
  //the new value is cheapest and if the value is also true then we want to throw out all the other options
  if(state->isTrueCost){
    while(!state->lazyList.empty())
      state->lazyList.pop();
  }
  putStateInHeap(state);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::insertLazyList(LazyAEGState* state, LazyAEGState* parent, int edgeCost, bool isTrueCost, EdgeType edgeType, int snap_midpoint){
  bool print = false; //state->id == 285566 || parent->id == 285566;
  if(state->v <= parent->v + edgeCost)
    return;
  else if(state->g <= parent->v + edgeCost){
    //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
    if(state->isTrueCost)
      return;
    //insert this guy into the lazy list
    LazyAEGListElement elem(parent,edgeCost,isTrueCost,edgeType,snap_midpoint);
    state->lazyList.push(elem);
  }
  else{//the new guy is the cheapest so far
    //should we save what was the previous best?
    if(!isTrueCost && //the better guy's cost is not for sure
       //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
       state->g < state->v){ //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
      //we save it by putting it in the lazy list
      LazyAEGListElement elem(state->best_parent, state->g - state->best_parent->v, state->isTrueCost, state->best_edge_type, state->snap_midpoint);
      state->lazyList.push(elem);
      //printf("save the previous best\n");
    }

    //the new guy is the cheapest
    state->g = parent->v + edgeCost;
    state->best_parent = parent;
    state->best_edge_type = edgeType;
    state->snap_midpoint = snap_midpoint;
    state->isTrueCost = isTrueCost;

    //the new value is cheapest and if the value is also true then we want to throw out all the other options
    if(isTrueCost){
      //printf("clear the lazy list\n");
      while(!state->lazyList.empty())
        state->lazyList.pop();
    }

    //this function puts the state into the heap (or updates the position) if we haven't expanded
    //if we have expanded, it will put the state in the incons list (if we haven't already)
    putStateInHeap(state);
  }
  if(print)
    printf("state->id=%d state->g=%d state->h=%d, parent->v=%d edgeCost=%d isTrueCost=%d\n",state->id,state->g,state->h,parent->v,edgeCost,isTrueCost);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::putStateInHeap(LazyAEGState* state){
  updateGoal(state);
  bool print = false; //state->id == 285566;
  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if(state->iteration_closed != search_iteration){
    CKey key;
    key.key[0] = state->g + int(eps * state->h);
    if(print)
      printf("put state in open with f %lu\n", key.key[0]);
    //if the state is already in the heap, just update its priority
    if(state->heapindex != 0)
      heap.updateheap(state,key);
    else //otherwise add it to the heap
      heap.insertheap(state,key);
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if(!state->in_incons){
    if(print)
      printf("put state in incons\n");
    incons.push_back(state);
    state->in_incons = true;
  }
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::updateGoal(LazyAEGState* state){
  if(egraph_mgr_->egraph_env_->isGoal(state->id) && state->isTrueCost && state->g < goal_state.g){
    //ROS_INFO("updating the goal state");
    goal_state.id = state->id;
    goal_state.g = state->g;
    goal_state.best_parent = state->best_parent;
    goal_state.best_edge_type = state->best_edge_type;
    goal_state.snap_midpoint = state->snap_midpoint;
    goal_state.isTrueCost = true;
  }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::ImprovePath(){

  //expand states until done
  int expands = 0;
  CKey min_key = heap.getminkeyheap();
  while(!heap.emptyheap() && 
        min_key.key[0] < INFINITECOST && 
        (goal_state.g > min_key.key[0] || !goal_state.isTrueCost) &&
        !outOfTime()){

    //get the state		
    LazyAEGState* state = (LazyAEGState*)heap.deleteminheap();

    if(state->v == state->g){
      printf("ERROR: consistent state is being expanded\n");
      printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
              state->id,state->v,state->g,state->isTrueCost,state->lazyList.size());
      std::cin.get();
    }

    if(state->isTrueCost){
      if (state->best_parent){
          /* Mike replacing victor's code
          Edge edge(state->best_parent->id, state->id);
          if (egraph_mgr_->snaps_.find(edge) != egraph_mgr_->snaps_.end()){
            evaluated_snaps++;
          }
          */
          if(state->best_edge_type == EdgeType::SNAP)
            evaluated_snaps++;
      }

      //mark the state as expanded
      state->v = state->g;
      state->expanded_best_parent = state->best_parent;
      state->expanded_best_edge_type = state->best_edge_type;
      state->expanded_snap_midpoint = state->snap_midpoint;
      state->iteration_closed = search_iteration;
      //expand the state
      expands++;
      ExpandState(state);
      if(expands%10000 == 0)
        printf("expands so far=%u\n", expands);
    }
    else //otherwise the state needs to be evaluated for its true cost
      EvaluateState(state);

    //get the min key for the next iteration
    min_key = heap.getminkeyheap();
  }

  search_expands += expands;
   
  if(goal_state.g == INFINITECOST && (heap.emptyheap() || min_key.key[0] >= INFINITECOST))
    return 0;//solution does not exists
  if(!heap.emptyheap() && goal_state.g > min_key.key[0])
    return 2; //search exited because it ran out of time
  printf("search exited with a solution for eps=%.2f\n", eps*params.epsE);
  if(goal_state.g < goal_state.v){
    goal_state.expanded_best_parent = goal_state.best_parent;
    goal_state.expanded_best_edge_type = goal_state.best_edge_type;
    goal_state.expanded_snap_midpoint = goal_state.snap_midpoint;
    goal_state.v = goal_state.g;
  }
  return 1;
}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::reconstructSuccs(LazyAEGState* state, 
                                               LazyAEGState*& next_state,
                                               vector<int>* wholePathIds,
                                               vector<int>* costs){
    //ROS_INFO("reconstruct with standard edge start");
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<bool> isTrueCost;
    if(bforwardsearch)
        environment_->GetLazySuccsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    else
        environment_->GetLazyPredsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    int actioncost = INFINITECOST;
    //ROS_INFO("reconstruct with standard edge %d\n",state->expanded_best_parent->id);
    for(unsigned int i=0; i<SuccIDV.size(); i++){
        //printf("  succ %d\n",SuccIDV[i]);
        if(SuccIDV[i] == state->id && CostV[i]<actioncost)
            actioncost = CostV[i];
    }
    if(actioncost == INFINITECOST){
        return false;
    } else {
        // remember we're starting from the goal and working backwards, so we
        // want to store the parents
        //ROS_INFO("good...");
        costs->push_back(actioncost);
        next_state = state->expanded_best_parent;
        wholePathIds->push_back(next_state->id);
        return true;
    }
}

template <typename HeuristicType>
vector<int> LazyAEGPlanner<HeuristicType>::GetSearchPath(int& solcost){
    clock_t reconstruct_t0 = clock();

    bool print = false;
    vector<int> wholePathIds;
    vector<int> costs;
    LazyAEGState* state;
    LazyAEGState* final_state;
    if(bforwardsearch){
        state = &goal_state;
        final_state = start_state;
    } else {
        state = start_state;
        final_state = &goal_state;
    } 

    wholePathIds.push_back(state->id);
    solcost = 0;
    int shortcut_count = 0;

    int shortcut_edges = 0;
    while(state->id != final_state->id){
        if(state->expanded_best_parent == NULL){
            printf("a state along the path has no parent!\n");
            assert(false);
        }
        if(state->v == INFINITECOST){
            printf("a state along the path has an infinite g-value!\n");
            printf("inf state = %d\n",state->id);
            assert(false);
        }

        LazyAEGState* next_state;
        if(state->expanded_best_edge_type == EdgeType::SNAP){
          assert(egraph_mgr_->reconstructSnap(state, next_state, &wholePathIds, &costs));
          if(print)
            ROS_INFO("snap edge %d %d %d", costs.back(), state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::NORMAL){
          bool ret = reconstructSuccs(state, next_state, &wholePathIds, &costs);
          assert(ret);
          if(print)
            ROS_INFO("normal edge %d %d %d", costs.back(), state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::DIRECT_SHORTCUT){
          int sc_cost;
          int before = wholePathIds.size();
          assert(egraph_mgr_->reconstructDirectShortcuts(state, next_state, &wholePathIds, &costs, shortcut_count, sc_cost));
          shortcut_edges += wholePathIds.size() - before;
          if(print)
            ROS_INFO("shortcut edge %d %d %d", sc_cost, state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::SNAP_DIRECT_SHORTCUT){
          int totalCost;
          int before = wholePathIds.size();
          assert(egraph_mgr_->reconstructSnapShortcut(state, next_state, &wholePathIds, &costs, totalCost));
          shortcut_edges += wholePathIds.size() - before;
          if(print)
            ROS_INFO("snap shortcut edge %d %d %d", totalCost, state->id, wholePathIds.back());
        }
        else
          assert(false);
        assert(next_state == state->expanded_best_parent);
        assert(wholePathIds.back() == state->expanded_best_parent->id);
        state = next_state;
    }
    percentFromShortcuts = double(shortcut_edges) / (wholePathIds.size()-1);

    //if we searched forward then the path reconstruction 
    //worked backward from the goal, so we have to reverse the path
    if(bforwardsearch){
        std::reverse(wholePathIds.begin(), wholePathIds.end());
        std::reverse(costs.begin(), costs.end());
    }
    solcost = std::accumulate(costs.begin(), costs.end(), 0);

    clock_t reconstruct_t1 = clock();
    reconstructTime = double(reconstruct_t1 - reconstruct_t0)/CLOCKS_PER_SEC;

    /*
    // if we're using lazy evaluation, we always want to feedback the path
    // regardless if it's valid
    if (params.feedback_path || params.use_lazy_validation){
        egraph_mgr_->storeLastPath(wholePathIds, costs);
    }
    if (params.use_lazy_validation){
        egraph_mgr_->feedbackLastPath();
    }
    */

    clock_t feedback_t0 = clock();
    if (params.feedback_path){
        egraph_mgr_->storeLastPath(wholePathIds, costs);
        egraph_mgr_->feedbackLastPath();
    }
    clock_t feedback_t1 = clock();
    feedbackPathTime = double(feedback_t1-feedback_t0)/CLOCKS_PER_SEC;
    //egraph_mgr_->printVector(wholePathIds);
    return wholePathIds;
}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::outOfTime(){
  //if the user has sent an interrupt signal we stop
  if(interruptFlag)
    return true;
  //if we are supposed to run until the first solution, then we are never out of time
  if(params.return_first_solution)
    return false;
  double time_used = double(clock() - TimeStarted)/CLOCKS_PER_SEC;
  if(time_used >= params.max_time)
    printf("out of max time\n");
  if(use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time)
    printf("used all repair time...\n");
  //we are out of time if:
         //we used up the max time limit OR
         //we found some solution and used up the minimum time limit
  return time_used >= params.max_time || 
         (use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::initializeSearch(){
  //it's a new search, so increment replan_number and reset the search_iteration
  replan_number++;
  search_iteration = 0;
  search_expands = 0;
  totalExpands = 0;
  succsClock = 0;
  shortcutClock = 0;
  snapClock = 0;
  heuristicClock = 0;
  reconstructTime = 0;
  feedbackPathTime = 0;

  //clear open list, incons list, and stats list
  heap.makeemptyheap();
  incons.clear();
  stats.clear();

  //initialize epsilon variable
  eps = params.initial_eps;
  eps_satisfied = INFINITECOST;

  //call get state to initialize the start and goal states
  //double t0 = ros::Time::now().toSec();
  //goal_state = GetState(goal_state_id);
  //if (!goal_state){
    //goal_state = GetState(goal_state_id);
  //} else {
    goal_state.g = INFINITECOST;
    goal_state.v = INFINITECOST;
    goal_state.iteration_closed = -1;
    goal_state.replan_number = replan_number;
    goal_state.best_parent = NULL;
    goal_state.expanded_best_parent = NULL;
    goal_state.best_edge_type = EdgeType::NONE;
    goal_state.expanded_best_edge_type = EdgeType::NONE;
    goal_state.snap_midpoint = -1;
    goal_state.expanded_snap_midpoint = -1;
    goal_state.heapindex = 0;
    goal_state.in_incons = false;
    goal_state.isTrueCost = true;

  //}
  start_state = GetState(start_state_id);

  // needed to add this because GetState calls the heuristic function on the
  // goal state before the heuristic has been initialized.
  goal_state.h = 0;

  //put start state in the heap
  start_state->g = 0;
  ROS_INFO("start state heuristic is %d", start_state->h);
  assert(start_state->h >= 0);
  CKey key;
  key.key[0] = eps*start_state->h;
  heap.insertheap(start_state, key);

  //ensure heuristics are up-to-date
  //environment_->EnsureHeuristicsUpdated((bforwardsearch==true));
  //printf("computing heuristic took %f sec\n", ros::Time::now().toSec()-t0);
}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::Search(vector<int>& pathIds, int& PathCost){
  CKey key;
  TimeStarted = clock();

  initializeSearch();

  //the main loop of ARA*
  while(eps_satisfied > params.final_eps && !outOfTime()){

    //run weighted A*
    clock_t before_time = clock();
    int before_expands = search_expands;
    //ImprovePath returns:
    //1 if the solution is found
    //0 if the solution does not exist
    //2 if it ran out of time
    int ret = ImprovePath();
    if(ret == 1) //solution found for this iteration
      eps_satisfied = eps;
    int delta_expands = search_expands - before_expands;
    double delta_time = double(clock()-before_time)/CLOCKS_PER_SEC;

    //print the bound, expands, and time for that iteration
    printf("bound=%f expands=%d cost=%d time=%.2f\n", 
        eps_satisfied*params.epsE, delta_expands, goal_state.g, delta_time);

    //update stats
    totalExpands += delta_expands;
    PlannerStats tempStat;
    tempStat.eps = eps_satisfied;
    tempStat.expands = delta_expands;
    tempStat.time = delta_time;
    tempStat.cost = goal_state.g;
    stats.push_back(tempStat);

    //no solution exists
    if(ret == 0){
      printf("Solution does not exist\n");
      return false;
    }

    //if we're just supposed to find the first solution
    //or if we ran out of time, we're done
    if(params.return_first_solution || ret == 2)
      break;

    prepareNextSearchIteration();
  }

  if(goal_state.g == INFINITECOST){
    printf("could not find a solution (ran out of time)\n");
    return false;
  }
  if(eps_satisfied == INFINITECOST)
    printf("WARNING: a solution was found but we don't have quality bound for it!\n");

  printf("solution found\n");
  pathIds = GetSearchPath(PathCost);

  return true;
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::prepareNextSearchIteration(){
  //decrease epsilon
  eps -= params.dec_eps;
  if(eps < params.final_eps)
    eps = params.final_eps;

  //dump the inconsistent states into the open list
  CKey key;
  while(!incons.empty()){
    LazyAEGState* s = incons.back();
    incons.pop_back();
    s->in_incons = false;
    key.key[0] = s->g + int(eps * s->h);
    heap.insertheap(s,key);
  }

  //recompute priorities for states in OPEN and reorder it
  for (int i=1; i<=heap.currentsize; ++i){
    LazyAEGState* state = (LazyAEGState*)heap.heap[i].heapstate;
    heap.heap[i].key.key[0] = state->g + int(eps * state->h); 
  }
  heap.makeheap();

  search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::interrupt(){
  interruptFlag = true;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams p){
  int solcost;
  return replan(solution_stateIDs_V, p, &solcost);
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(int start, vector<int>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  set_start(start);
  //set_goal(goal);
  return replan(solution_stateIDs_V, p, solcost);
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  clock_t replan_t0 = clock();
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;
  interruptFlag = false;

  egraph_mgr_->setEpsE(p.epsE);
  set_goal();

  if(start_state_id < 0){
    printf("ERROR searching: no start state set\n");
    return 0;
  }
  if (egraph_mgr_->egraph_env_->isGoal(start_state_id)){
    ROS_WARN("start is goal! nothing interesting returned");
    return true;
  }

  //plan
  vector<int> pathIds; 
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  start_state_id = -1;
  //goal_state_id = -1;

  clock_t replan_t1 = clock();
  totalPlanTime = double(replan_t1-replan_t0)/CLOCKS_PER_SEC;

  printf("\n---------------------------------------------------------------\n");
  if(solnFound)
    printf("Solution found!\n");
  else
    printf("Solution not found...\n");
  printf("total time=%.2f total time without counting adding new path=%.2f\n", 
          totalPlanTime, totalPlanTime-feedbackPathTime);
  printf("total expands=%d solution cost=%d\n", 
          totalExpands, goal_state.g);
  printf("time breakdown: heuristic set goal  = %.2f\n", heuristicSetGoalTime);
  printf("                heuristic           = %.2f\n", double(heuristicClock)/CLOCKS_PER_SEC);
  printf("                generate successors = %.2f\n", double(succsClock)/CLOCKS_PER_SEC);
  printf("                shortcuts           = %.2f\n", double(shortcutClock)/CLOCKS_PER_SEC);
  printf("                snaps               = %.2f\n", double(snapClock)/CLOCKS_PER_SEC);
  printf("                path reconstruction = %.2f\n", reconstructTime);
  printf("                feedback path       = %.2f\n", feedbackPathTime);
  printf("---------------------------------------------------------------\n\n");

  stat_map_["solution_found"] = solnFound;
  stat_map_["solution_bound"] = params.epsE*params.initial_eps;
  stat_map_["total_time"] = totalPlanTime;
  stat_map_["total_time_without_feedback"] = totalPlanTime-feedbackPathTime;
  stat_map_["expands"] = totalExpands;
  stat_map_["solution_cost"] = goal_state.g;
  stat_map_["heuristic_set_goal_time"] = heuristicSetGoalTime;
  stat_map_["heuristic_time"] = double(heuristicClock)/CLOCKS_PER_SEC;
  stat_map_["generate_successors_time"] = double(succsClock)/CLOCKS_PER_SEC;
  stat_map_["shortcuts_time"] = double(shortcutClock)/CLOCKS_PER_SEC;
  stat_map_["snap_time"] = double(snapClock)/CLOCKS_PER_SEC;
  stat_map_["path_reconstruction_time"] = reconstructTime;
  stat_map_["feedback_path_time"] = feedbackPathTime;
  stat_map_["percent_from_shortcuts"] = percentFromShortcuts;
  //stat_map_["shortcut_bfs"] = egraph_mgr_->getStats().get_direct_shortcut_time;
  //stat_map_["egraph_dijkstra"] = egraph_mgr_->getStats().shortest_path_time;
         
  return (int)solnFound;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::set_goal(){
  /*
  printf("planner: setting goal to %d\n", id);
  if(bforwardsearch)
    goal_state_id = id;
  else
    start_state_id = id;
  */
  if (!params.use_lazy_validation){
      ROS_INFO("fully validating egraph");
      egraph_mgr_->validateEGraph();
  }
  clock_t t0 = clock();
  egraph_mgr_->setGoal();
  clock_t t1 = clock();
  heuristicSetGoalTime = double(t1-t0)/CLOCKS_PER_SEC;

  if (!params.use_lazy_validation){
      egraph_mgr_->initEGraph();
  }
  return 1;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::set_start(int id){
  //printf("planner: setting start to %d\n", id);
  //if(bforwardsearch)
    start_state_id = id;
  //else
    //goal_state_id = id;
  return 1;
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::feedback_last_path(){
    egraph_mgr_->feedbackLastPath();
    printf("validitycheck time=%.3f feedbacktime %.3f errorcheck time=%.3f\n",
            egraph_mgr_->getStats().egraph_validity_check_time,
            egraph_mgr_->getStats().feedback_time,
            egraph_mgr_->getStats().error_check_time);
}


//---------------------------------------------------------------------------------------------------------


template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::get_search_stats(vector<PlannerStats>* s){
  s->clear();
  s->reserve(stats.size());
  for(unsigned int i=0; i<stats.size(); i++){
    s->push_back(stats[i]);
  }
}

