#include<navigation_xy/navigation_xy.h>

using namespace std;

EGraphXY::EGraphXY(double map_cell_res){
  map_cell_res_ = map_cell_res;
}

bool EGraphXY::snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
  return false;
}

bool EGraphXY::getCoord(int id, vector<double>& coord){
  EnvNAV2DHashEntry_t* hashEntry = EnvNAV2D.StateID2CoordTable[id];
  coord.clear();
  coord.push_back(hashEntry->X);
  coord.push_back(hashEntry->Y);
  return true;
}

int EGraphXY::getStateID(const vector<double>& coord){
  return GetStateFromCoord(coord[0],coord[1]);
}

bool EGraphXY::isGoal(int id){
  return id == EnvNAV2D.goalstateid;
}

void EGraphXY::projectToHeuristicSpace(const vector<double>& coord, vector<double>& dp) const{
  dp = coord;
}

void EGraphXY::projectGoalToHeuristicSpace(vector<double>& dp) const{
  EnvNAV2DHashEntry_t* hashEntry = EnvNAV2D.StateID2CoordTable[EnvNAV2D.goalstateid];
  dp.clear();
  dp.push_back(hashEntry->X);
  dp.push_back(hashEntry->Y);
}

void EGraphXY::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(2);
  d[0] = c[0];
  d[1] = c[1];
}

void EGraphXY::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(2);
  c[0] = d[0];
  c[1] = d[1];
}

bool EGraphXY::isValidEdge(const vector<double>& coord, const vector<double>& coord2, bool& change_cost, int& cost){
  int id1 = getStateID(coord);
  int id2 = getStateID(coord2);

  vector<int> children;
  vector<int> costs;

  GetSuccs(id1, &children, &costs);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id2){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }
  GetSuccs(id2, &children, &costs);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id1){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }

  change_cost = false;
  return false;
}

bool EGraphXY::isValidVertex(const vector<double>& coord){
  return IsValidCell(coord[0],coord[1]);
}

visualization_msgs::MarkerArray EGraphXY::stateToVisualizationMarker(vector<double> coord){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = coord[0]*map_cell_res_;
  marker.pose.position.y = coord[1]*map_cell_res_;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);
  return ma;
}

visualization_msgs::MarkerArray EGraphXY::stateToDetailedVisualizationMarker(vector<double> coord){
  return stateToVisualizationMarker(coord);
}

visualization_msgs::MarkerArray EGraphXY::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.01;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = coord[0]*map_cell_res_;
  p.y = coord[1]*map_cell_res_;
  p.z = 0;
  marker.points.push_back(p);
  p.x = coord2[0]*map_cell_res_;
  p.y = coord2[1]*map_cell_res_;
  marker.points.push_back(p);

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);
  return ma;
}

