#ifndef EGRAPH_3D_GRID_HEURISTIC_H
#define EGRAPH_3D_GRID_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include <map>

class EGraph3dGridHeuristic : public EGraphHeuristic<std::vector<int> >{
  public:
    EGraph3dGridHeuristic(const EGraphable<std::vector<int> >& env, int size_x, int size_y, int size_z, int move_cost);
    void setGrid(const std::vector<std::vector<std::vector<bool> > >& grid);
    void setGoal(const std::vector<int>& goal);
    int getHeuristic(const std::vector<int>& coord);
    void getEGraphVerticesWithSameHeuristic(const std::vector<int>& coord, std::vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, std::vector<EGraph::EGraphVertex*>& shortcuts);
    virtual void resetShortcuts();
    //void setEpsE(double e){ epsE_ = e; inflated_cost_1_move_ = cost_1_move_ * epsE_;};

  protected:
    class EGraph3dGridHeuristicCell: public AbstractSearchState{
      public:
        EGraph3dGridHeuristicCell(){
          open_iteration = 0;
          closed_iteration = 0;
        };
        ~EGraph3dGridHeuristicCell(){};
          
        int open_iteration;
        int closed_iteration;
        int id;
        int cost;
        std::vector<EGraph::EGraphVertex*> egraph_vertices;
    };

    int iteration_;

    int sizex_;
    int sizey_;
    int sizez_;
    int width_;
    int height_;
    int length_;
    int planeSize_;
    int gridSize_;
    int cost_1_move_;
    int inflated_cost_1_move_;
    CHeap heap;
    CHeap sc_heap;
    std::vector<int> goal_dp_;
    
    std::vector<bool> empty_components_;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    std::vector<EGraph3dGridHeuristicCell> heur;
    std::vector<EGraph3dGridHeuristicCell> sc;
    const EGraphable<std::vector<int> >& env_;
};

#endif
