#ifndef EGRAPH_EUCLIDEAN_HEURISTIC_H
#define EGRAPH_EUCLIDEAN_HEURISTIC_H

#include<egraphs/egraph_heuristic.h>
#include<egraphs/egraph.h>
#include<sbpl/headers.h>
#include<map>
#include<vector>

class EGraphEuclideanHeuristic : public EGraphHeuristic<std::vector<double> >{
  public:
    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, double distance_inflation);
    EGraphEuclideanHeuristic(const EGraphable<std::vector<double> >& env, const std::vector<double>& element_diff_inflation);
    void setGoal(const std::vector<double>& goal);
    int getHeuristic(const std::vector<double>& coord);
    void getEGraphVerticesWithSameHeuristic(const std::vector<double>& coord, std::vector<EGraph::EGraphVertex*>& vertices);
    void runPrecomputations();
    void getDirectShortcut(int component, std::vector<EGraph::EGraphVertex*>& shortcuts);
    void resetShortcuts();
    inline int euclideanDistance(const std::vector<double>& c1, const std::vector<double>& c2);

  protected:

    class EGraphEuclideanState : public AbstractSearchState{
      public:
        int id;
        int g;
        std::vector<double> coord;
    };

    CHeap heap;

    double dist_inflation;
    std::vector<double> inflation;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    const EGraphable<std::vector<double> >& env_;
    std::vector<double> goal_;

    std::vector<EGraphEuclideanState> verts;
};

#endif
