#ifndef EGRAPH_MARKER_MAKER_H
#define EGRAPH_MARKER_MAKER_H

#include<vector>
#include<visualization_msgs/MarkerArray.h>

class EGraphMarkerMaker{
  public:
    //Given a state vector, return a MarkerArray that represents the state in a compact way.
    //Every state in the E-Graph will be visualized at the same time using this function so
    //the marker should be small (typically a down projection) like a sphere otherwise
    //you won't be able to see anything. You will be able to click on these compact markers
    //to display a more detailed one on demand.
    virtual visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord) = 0;

    //Given two state vectors, return a MarkerArray that represents the edge in a compact way.
    //Every edge in the E-Graph will be visualized at the same time using this function so
    //the marker should be small (typically a down projection) like a line, otherwise
    //you won't be able to see anything. 
    virtual visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2) = 0;

    //Given a state vector, return a MarkerArray that represents the state in a very detailed way,
    //such as all the robot's meshes in the proper configuration. Don't worry about cluttering
    //the display. These markers show up on demand by clicking one of the compact markers
    //(stateToVisualizationMarker). 
    virtual visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord) = 0;
};

#endif

