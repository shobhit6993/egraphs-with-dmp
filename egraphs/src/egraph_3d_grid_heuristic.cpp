#include<egraphs/egraph_3d_grid_heuristic.h>

using namespace std;

#define HEUR_XYZ2ID(x,y,z) ((z + 1) * planeSize_ + (y + 1) * width_ + (x + 1))

EGraph3dGridHeuristic::EGraph3dGridHeuristic(const EGraphable<vector<int> >& env, 
                                             int size_x, int size_y, int size_z, 
                                             int move_cost): env_(env){
  sizex_ = size_x;
  sizey_ = size_y;
  sizez_ = size_z;
  cost_1_move_ = move_cost;

  iteration_ = 0;

  width_ = sizex_ + 2;
  height_ = sizey_ + 2;
  length_ = sizez_ + 2;
  planeSize_ = width_ * height_;
  gridSize_ = planeSize_ * length_;
  ROS_INFO("sizes: x=%d y=%d z=%d plane=%d grid=%d\n",sizex_,sizey_,sizez_,planeSize_,gridSize_);

  heur.resize(gridSize_);
  sc.resize(gridSize_);
  for (int i = 0; i < gridSize_; i++) {
    int x = i % width_, y = i / width_ % height_, z = i / planeSize_;
    if (x == 0 || x == width_ - 1 || y == 0 || y == height_ - 1 || z == 0 || z == length_ - 1){
      heur[i].cost = -1;
      sc[i].cost = -1;
    }
    else{
      heur[i].cost = INFINITECOST;
      sc[i].cost = INFINITECOST;
    }
  }
  for(int i=0; i<gridSize_; i++){
    heur[i].id = i;
    sc[i].id = i;
  }
}

void EGraph3dGridHeuristic::setGrid(const vector<vector<vector<bool> > >& grid){
  if(grid.size() != (unsigned int)(sizex_) ||
     grid.front().size() != (unsigned int)(sizey_) ||
     grid.front().front().size() != (unsigned int)(sizez_)){
    ROS_ERROR("[EGraph3dGridHeuristic] The dimensions provided in the constructor don't match the given grid.");
    return;
  }

  for(unsigned int x=0; x<grid.size(); x++){
    for(unsigned int y=0; y<grid[x].size(); y++){
      for(unsigned int z=0; z<grid[x][y].size(); z++){
        int id = HEUR_XYZ2ID(x,y,z);
        if(grid[x][y][z]){
          heur[id].cost = -1;
          sc[id].cost = -1;
        }
        else{
          heur[id].cost = INFINITECOST;
          sc[id].cost = INFINITECOST;
        }
      }
    }
  }

}

void EGraph3dGridHeuristic::getEGraphVerticesWithSameHeuristic(const vector<int>& coord, vector<EGraph::EGraphVertex*>& vertices){
  vertices.clear();
  //printf("verts in cell: %d\n",heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices.size());
  vertices = heur[HEUR_XYZ2ID(coord[0],coord[1],coord[2])].egraph_vertices;
}

void EGraph3dGridHeuristic::runPrecomputations(){
  //ROS_INFO("begin precomputations");
  //refill the cell to egraph vertex mapping
  //clock_t time = clock();
  for(int i=0; i<gridSize_; i++){
    heur[i].egraph_vertices.clear();
    sc[i].egraph_vertices.clear();
  }

  empty_components_.clear();
  // if false, 
  empty_components_.resize(eg_->getNumComponents(), false);
  vector<int> dp;
  vector<double> c_coord;
  //ROS_INFO("down project edges...");
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    bool valid = false;
    for(unsigned int a=0; a<eg_->id2vertex[i]->valid.size(); a++)
      valid |= eg_->id2vertex[i]->valid[a];
    if(!valid){
      empty_components_[eg_->id2vertex[i]->component] = true;
      continue;
    }

    eg_->discToCont(eg_->id2vertex[i],c_coord);
    //ROS_INFO("size of coord %d",c_coord.size());
    env_.projectToHeuristicSpace(c_coord,dp);
    if(dp[0] > sizex_ ||
        dp[1] > sizey_ ||
        dp[2] > sizez_){
      ROS_WARN("edge out of bounds %d",eg_->id2vertex[i]->id);
      continue;
    }
    //ROS_INFO("size of coord %d",dp.size());
    //ROS_INFO("coord %d %d %d",dp[0],dp[1],dp[2]);
    heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices.push_back(eg_->id2vertex[i]);
    sc[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices.push_back(eg_->id2vertex[i]);
    //ROS_INFO("push_back");
  }
  shortcut_cache_.clear();
  shortcut_cache_.resize(eg_->getNumComponents(), NULL);
  //ROS_INFO("precomp time took %f", double(clock()-time)/CLOCKS_PER_SEC);
  //ROS_INFO("done precomputations");
}

void EGraph3dGridHeuristic::resetShortcuts(){
  //ROS_ERROR("begin resetShortcuts");
  for(int i=0; i<gridSize_; i++){
    sc[i].id = i;
    sc[i].heapindex = 0;
    sc[i].closed_iteration--;
    if(sc[i].cost!=-1)
      sc[i].cost = INFINITECOST;
  }
  sc_heap.makeemptyheap();
  int id = HEUR_XYZ2ID(goal_dp_[0],goal_dp_[1],goal_dp_[2]);
  CKey key;
  key.key[0] = 0;
  sc_heap.insertheap(&sc[id],key);
  sc[id].cost = 0;
  shortcut_cache_.clear();
  shortcut_cache_.resize(eg_->getNumComponents(), NULL);
}

void EGraph3dGridHeuristic::setGoal(const vector<int>& goal){
  //ROS_ERROR("begin setGoal");
  iteration_++;
  /*
  //clear the heur data structure
  clock_t t0 = clock();
  for(int i=0; i<gridSize_; i++){
    heur[i].id = i;
    heur[i].heapindex = 0;
    heur[i].closed = false;
    if(heur[i].cost!=-1)
      heur[i].cost = INFINITECOST;
  }
  printf("clear heur grid %f\n",double(clock()-t0)/CLOCKS_PER_SEC);
  t0 = clock();
  for(int i=0; i<gridSize_; i++){
    sc[i].id = i;
    sc[i].heapindex = 0;
    sc[i].closed = false;
    if(sc[i].cost!=-1)
      sc[i].cost = INFINITECOST;
  }
  printf("clear sc grid %f\n",double(clock()-t0)/CLOCKS_PER_SEC);
  */
  
  vector<int> dp;
  if(goal.empty()){
    dp = goal_dp_;
  }
  else{
    dp = goal;
    goal_dp_ = dp;
  }
  CKey key;
  key.key[0] = 0;
  heap.makeemptyheap();
  int id = HEUR_XYZ2ID(dp[0],dp[1],dp[2]);
  heur[id].cost = 0;
  heur[id].heapindex = 0;
  heur[id].open_iteration = iteration_;
  heap.insertheap(&heur[id],key);

  sc_heap.makeemptyheap();
  id = HEUR_XYZ2ID(dp[0],dp[1],dp[2]);
  sc[id].cost = 0;
  sc[id].heapindex = 0;
  sc[id].open_iteration = iteration_;
  sc_heap.insertheap(&sc[id],key);

  inflated_cost_1_move_ = cost_1_move_ * epsE_;
  shortcut_cache_.clear();
  shortcut_cache_.resize(eg_->getNumComponents(), NULL);
}

#define HEUR_SUCCESSOR(offset){                           \
  if(heur[id + (offset)].cost != -1){                     \
    if(heur[id + (offset)].open_iteration != iteration_){ \
      heur[id + (offset)].open_iteration = iteration_;    \
      heur[id + (offset)].cost = currentCost;             \
      heur[id + (offset)].heapindex = 0;                  \
      heap.insertheap(&heur[id + (offset)],key);          \
    }                                                     \
    else if((heur[id + (offset)].cost > currentCost)){    \
      heap.updateheap(&heur[id + (offset)],key);          \
      heur[id + (offset)].cost = currentCost;             \
    }                                                     \
  }                                                       \
}

int EGraph3dGridHeuristic::getHeuristic(const vector<int>& coord){
  if(coord[0] > sizex_ ||
     coord[1] > sizey_ ||
     coord[2] > sizez_){
    ROS_ERROR("out of bounds heuristic request: %d %d %d -> %d\n",coord[0],coord[1],coord[2],HEUR_XYZ2ID(coord[0],coord[1],coord[2]));
    exit(1);
    return INFINITECOST;
  }

  EGraph3dGridHeuristicCell* cell = &heur[HEUR_XYZ2ID(coord[0],coord[1],coord[2])];

  if(cell->cost==-1)
    return INFINITECOST;
  
  vector<int> dp(3,0);
  CKey key;
  //compute distance from H to all cells and note for each cell, what node in H was the closest
  while(!heap.emptyheap() && cell->closed_iteration != iteration_){
    EGraph3dGridHeuristicCell* state = (EGraph3dGridHeuristicCell*)heap.deleteminheap();
    int id = state->id;
    state->closed_iteration = iteration_;
    int oldCost = state->cost;
    int currentCost = oldCost + inflated_cost_1_move_;
    key.key[0] = currentCost;

    HEUR_SUCCESSOR(-width_);              //-y
    HEUR_SUCCESSOR(1);                    //+x
    HEUR_SUCCESSOR(width_);               //+y
    HEUR_SUCCESSOR(-1);                   //-x
    HEUR_SUCCESSOR(-width_-1);            //-y-x
    HEUR_SUCCESSOR(-width_+1);            //-y+x
    HEUR_SUCCESSOR(width_+1);             //+y+x
    HEUR_SUCCESSOR(width_-1);             //+y-x
    HEUR_SUCCESSOR(planeSize_);           //+z
    HEUR_SUCCESSOR(-width_+planeSize_);   //+z-y
    HEUR_SUCCESSOR(1+planeSize_);         //+z+x
    HEUR_SUCCESSOR(width_+planeSize_);    //+z+y
    HEUR_SUCCESSOR(-1+planeSize_);        //+z-x
    HEUR_SUCCESSOR(-width_-1+planeSize_); //+z-y-x
    HEUR_SUCCESSOR(-width_+1+planeSize_); //+z-y+x
    HEUR_SUCCESSOR(width_+1+planeSize_);  //+z+y+x
    HEUR_SUCCESSOR(width_-1+planeSize_);  //+z+y-x
    HEUR_SUCCESSOR(-planeSize_);          //-z
    HEUR_SUCCESSOR(-width_-planeSize_);   //-z-y
    HEUR_SUCCESSOR(1-planeSize_);         //-z+x
    HEUR_SUCCESSOR(width_-planeSize_);    //-z+y
    HEUR_SUCCESSOR(-1-planeSize_);        //-z-x
    HEUR_SUCCESSOR(-width_-1-planeSize_); //-z-y-x
    HEUR_SUCCESSOR(-width_+1-planeSize_); //-z-y+x
    HEUR_SUCCESSOR(width_+1-planeSize_);  //-z+y+x
    HEUR_SUCCESSOR(width_-1-planeSize_);  //-z+y-x

    vector<double> c_coord;
    for(unsigned int i=0; i<state->egraph_vertices.size(); i++){
      for(unsigned int j=0; j<state->egraph_vertices[i]->neighbors.size(); j++){
        if(!state->egraph_vertices[i]->valid[j])
          continue;
        eg_->discToCont(state->egraph_vertices[i]->neighbors[j],c_coord);
        env_.projectToHeuristicSpace(c_coord,dp);
        EGraph3dGridHeuristicCell* cell = &heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])];
        int newCost = oldCost + state->egraph_vertices[i]->costs[j];
        if(cell->open_iteration != iteration_){
          cell->open_iteration = iteration_;
          cell->cost = newCost;
          cell->heapindex = 0;
          key.key[0] = newCost;
          heap.insertheap(cell,key);
        }
        else if(cell->cost > newCost){
          cell->cost = newCost;
          key.key[0] = newCost;
          heap.updateheap(cell,key);
        }
      }
    }

  }
  return cell->cost;
}

#define SHORTCUT_SUCCESSOR(offset){                     \
  if(sc[id + (offset)].cost != -1){                     \
    if(sc[id + (offset)].open_iteration != iteration_){ \
      sc[id + (offset)].open_iteration = iteration_;    \
      sc[id + (offset)].cost = currentCost;             \
      sc[id + (offset)].heapindex = 0;                  \
      sc_heap.insertheap(&sc[id + (offset)],key);       \
    }                                                   \
    else if((sc[id + (offset)].cost > currentCost)){    \
      sc_heap.updateheap(&sc[id + (offset)],key);       \
      sc[id + (offset)].cost = currentCost;             \
    }                                                   \
  }                                                     \
}

// 3d breadth first search from goal to all other states until desired component
//
void EGraph3dGridHeuristic::getDirectShortcut(int component, vector<EGraph::EGraphVertex*>&     shortcuts){
  //we can assume that we would not be called if we have already discovered that component

  shortcuts.clear();
  if (shortcut_cache_[component]){
    shortcuts.push_back(shortcut_cache_[component]);
    return;
  }

  // if we have already determined that this is an empty component, skip
  if (empty_components_[component]){
    return;
  }

  CKey key;
  int counter = 0;
  //compute distance from H to all cells and note for each cell, what node in H was the closest
  while(!sc_heap.emptyheap() && shortcuts.empty()){
    EGraph3dGridHeuristicCell* state = (EGraph3dGridHeuristicCell*)sc_heap.deleteminheap();
    int id = state->id;
    state->closed_iteration = iteration_;
    int oldCost = state->cost;
    int currentCost = oldCost + inflated_cost_1_move_;
    key.key[0] = currentCost;

    SHORTCUT_SUCCESSOR(-width_);              //-y
    SHORTCUT_SUCCESSOR(1);                    //+x
    SHORTCUT_SUCCESSOR(width_);               //+y
    SHORTCUT_SUCCESSOR(-1);                   //-x
    SHORTCUT_SUCCESSOR(-width_-1);            //-y-x
    SHORTCUT_SUCCESSOR(-width_+1);            //-y+x
    SHORTCUT_SUCCESSOR(width_+1);             //+y+x
    SHORTCUT_SUCCESSOR(width_-1);             //+y-x
    SHORTCUT_SUCCESSOR(planeSize_);           //+z
    SHORTCUT_SUCCESSOR(-width_+planeSize_);   //+z-y
    SHORTCUT_SUCCESSOR(1+planeSize_);         //+z+x
    SHORTCUT_SUCCESSOR(width_+planeSize_);    //+z+y
    SHORTCUT_SUCCESSOR(-1+planeSize_);        //+z-x
    SHORTCUT_SUCCESSOR(-width_-1+planeSize_); //+z-y-x
    SHORTCUT_SUCCESSOR(-width_+1+planeSize_); //+z-y+x
    SHORTCUT_SUCCESSOR(width_+1+planeSize_);  //+z+y+x
    SHORTCUT_SUCCESSOR(width_-1+planeSize_);  //+z+y-x
    SHORTCUT_SUCCESSOR(-planeSize_);          //-z
    SHORTCUT_SUCCESSOR(-width_-planeSize_);   //-z-y
    SHORTCUT_SUCCESSOR(1-planeSize_);         //-z+x
    SHORTCUT_SUCCESSOR(width_-planeSize_);    //-z+y
    SHORTCUT_SUCCESSOR(-1-planeSize_);        //-z-x
    SHORTCUT_SUCCESSOR(-width_-1-planeSize_); //-z-y-x
    SHORTCUT_SUCCESSOR(-width_+1-planeSize_); //-z-y+x
    SHORTCUT_SUCCESSOR(width_+1-planeSize_);  //-z+y+x
    SHORTCUT_SUCCESSOR(width_-1-planeSize_);  //-z+y-x

    for(size_t i=0; i<state->egraph_vertices.size(); i++){
      int comp_num = state->egraph_vertices[i]->component;
      if(!shortcut_cache_[comp_num]){
        shortcut_cache_[comp_num] = state->egraph_vertices[i];
        bool valid = 0;
        for (size_t j = 0; j < state->egraph_vertices[i]->valid.size(); j++){
            valid |= state->egraph_vertices[i]->valid[j];
        }
        assert(valid == true);
        if (comp_num == component){
            // remember, when this gets filled in, it also breaks out of the
            // while loop
            shortcuts.push_back(state->egraph_vertices[i]);
            break;
        }
      }
    }
    counter++;
  }
  //ROS_INFO("number of shortcuts returned %lu", shortcuts.size());
  //ROS_INFO("number of shortcut expands: %d", counter);

  if (shortcuts.empty()){
      int count=0;
      for (size_t i=0; i < eg_->id2vertex.size(); i++){
          if (eg_->id2vertex[i]->component == component){
              count++;
          }
      }

      int valid_count = 0;
      int invalid_count = 0;
      ROS_ERROR("component %d has no shortcuts but has %d vertices!", component, count);
      for (size_t i=0; i < eg_->id2vertex.size(); i++){
          if (eg_->id2vertex[i]->component == component){
              for (size_t j=0; j < eg_->id2vertex[i]->neighbors.size(); j++){
                  vector<double> c;
                  eg_->discToCont(eg_->id2vertex[i]->neighbors[j], c);
                  if (eg_->id2vertex[i]->valid[j]){
                      valid_count++;
                  } else {
                      invalid_count++;
                  }
              }
          }
      }

      ROS_ERROR("has %d valid edges and %d invalid edges",valid_count, invalid_count);

      assert(false);
  }
}


