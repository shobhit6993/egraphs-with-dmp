#ifndef NAVIGATION_XY_H
#define NAVIGATION_XY_H

#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<egraphs/egraph_euclidean_heuristic.h>
#include<egraphs/egraph_discretize.h>
#include<sbpl/headers.h>
#include<egraph_vis/egraph_visualizer.h>

class EGraphXY: public EnvironmentNAV2D, public EGraphable<std::vector<double> >, public EGraphMarkerMaker, public EGraphDiscretize{
  public:
    EGraphXY(double map_cell_res);
    bool snap(const std::vector<double>& from, const std::vector<double>& to, int& id, int& cost);
    virtual bool getCoord(int id, std::vector<double>& coord);
    virtual int getStateID(const std::vector<double>& coord);
    virtual bool isGoal(int id);
    void projectToHeuristicSpace(const std::vector<double>& coord, std::vector<double>& dp) const;
    void projectGoalToHeuristicSpace(std::vector<double>& dp) const;
    void contToDisc(const std::vector<double>& c, std::vector<int>& d);
    void discToCont(const std::vector<int>& d, std::vector<double>& c);
    virtual bool isValidEdge(const std::vector<double>& coord, const std::vector<double>& coord2, bool& change_cost, int& cost);
    virtual bool isValidVertex(const std::vector<double>& coord);
    visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2);

  private:
    double map_cell_res_;
};

#endif
