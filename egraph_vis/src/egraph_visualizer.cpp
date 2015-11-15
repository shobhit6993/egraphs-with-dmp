#include<egraph_vis/egraph_visualizer.h>

using namespace std;

EGraphVisualizer::EGraphVisualizer(EGraph* eg, EGraphMarkerMaker* converter){
  eg_ = eg;
  converter_ = converter;

  //set up interactive marker server
  server_.reset(new interactive_markers::InteractiveMarkerServer("EGraph","",true));
  ros::Duration(0.1).sleep();
  menu_handler_.insert("Show/Hide Neighborhood", boost::bind(&EGraphVisualizer::processFeedback, this, _1));
  menu_handler_.insert("Show/Hide Shortcuts", boost::bind(&EGraphVisualizer::processFeedback, this, _1));
}

EGraphVisualizer::~EGraphVisualizer(){
  server_.reset();
}

void EGraphVisualizer::visualize(){
  server_->clear();
  vis_table_.clear();
  vis_table_.resize(eg_->id2vertex.size());
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    EGraph::EGraphVertex* v = eg_->id2vertex[i];

    bool valid = false;
    for(unsigned int a=0; a<v->valid.size(); a++)
      valid |= v->valid[a];
    if(!valid)
      continue;

    addState(v,false);

    vector<double> coord;
    eg_->discToCont(v,coord);
    for(unsigned int j=0; j<v->neighbors.size(); j++){
      if(!v->valid[j])
        continue;
      EGraph::EGraphVertex* u = v->neighbors[j];
      if(v->id<u->id){
        vector<double> coord2;
        eg_->discToCont(u,coord2);
        visualization_msgs::MarkerArray m = converter_->edgeToVisualizationMarker(coord,coord2);

        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = m.markers.front().header.frame_id;
        int_marker.pose = m.markers.front().pose;
        int_marker.scale = 1;
        int_marker.description = "";
        int_marker.name = (string("egraph_edge_") + boost::lexical_cast<string>(i) + string("_") + boost::lexical_cast<string>(j)).c_str();

        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

        for(unsigned int i=0; i<m.markers.size(); i++)
          control.markers.push_back(m.markers[i]);
        control.always_visible = true;
        int_marker.controls.push_back(control);

        server_->insert(int_marker);
      }
    }
  }
  server_->applyChanges();
}

void EGraphVisualizer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  //parse the marker name
  char name[128];
  strncpy(name,feedback->marker_name.c_str(),sizeof(name));
  char* pch = strtok(name,"_");
  if(!pch || strcmp(pch,"egraph")!=0)
    return;
  //ok, this is an egraph marker...
  
  pch = strtok (NULL, "_");
  if(!pch || strcmp(pch,"vertex")!=0)
    return;
  //and it's a vertex...

  pch = strtok (NULL, "_");
  if(!pch)
    return;
  int id = atoi(pch);
  //we got the vertex id

  //if there are any more tokens, then it must be a "detailed" tag
  pch = strtok (NULL, "_");
  bool isDetailed = pch;

  //TODO: in a thread safe manner, check the version number of the egraph and compare
  //it to the version number from when we last visualized. make sure they are the same
  //so that we know that this id is still valid!
  EGraph::EGraphVertex* v = eg_->id2vertex[id];
  //we got the vertex from the e-graph
  
  
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
    if(feedback->menu_entry_id == 1){
      //Show/Hide Neighborhood
      if(vis_table_[v->id].neighbors){
        //hide
        for(unsigned int i=0; i<v->neighbors.size(); i++)
          server_->erase((string("egraph_neighbor_")+boost::lexical_cast<string>(v->id)+string("_")+boost::lexical_cast<string>(i)).c_str());
        vis_table_[v->id].neighbors = false;
      }
      else{
        //show
        for(unsigned int i=0; i<v->neighbors.size(); i++)
          addNeighbor(v,i);
        vis_table_[v->id].neighbors = true;
      }
    }
    else if(feedback->menu_entry_id == 2){
      //Show/Hide Shortcuts
      
    }
  }
  else if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN){
    //Draw the detailed version of this vertex (this has the same menu controls as the regular vertex)
    if(isDetailed){
      //if this is a detailed vertex, delete it from the server
      server_->erase(feedback->marker_name);
    }
    else{
      //we left clicked on a regular vertex
      if(vis_table_[v->id].detailed){
        //the detailed one was already drawn so delete it!
        string detailed_name = feedback->marker_name + string("_detailed");
        server_->erase(detailed_name);
        vis_table_[v->id].detailed = false;
      }
      else{
        //we will draw the detailed version of the state
        addState(v,true);
        vis_table_[v->id].detailed = true;
      }
    }
  }
  server_->applyChanges();
}

void EGraphVisualizer::addState(EGraph::EGraphVertex* v, bool detailed){
  vector<double> coord;
  eg_->discToCont(v,coord);
  visualization_msgs::MarkerArray m;
  if(detailed)
    m = converter_->stateToDetailedVisualizationMarker(coord);
  else
    m = converter_->stateToVisualizationMarker(coord);
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = m.markers.front().header.frame_id;
  int_marker.pose = m.markers.front().pose;
  int_marker.scale = 1;
  int_marker.description = "";
  if(detailed)
    int_marker.name = (string("egraph_vertex_") + boost::lexical_cast<string>(v->id) + string("_detailed")).c_str();
  else
    int_marker.name = (string("egraph_vertex_") + boost::lexical_cast<string>(v->id)).c_str();

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  if(detailed)
    control.name = (string("egraph_menu_control_") + boost::lexical_cast<string>(v->id) + string("_detailed")).c_str();
  else
    control.name = (string("egraph_menu_control_") + boost::lexical_cast<string>(v->id)).c_str();

  for(unsigned int i=0; i<m.markers.size(); i++)
    control.markers.push_back(m.markers[i]);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&EGraphVisualizer::processFeedback, this, _1));
  menu_handler_.apply(*server_, int_marker.name);
}

void EGraphVisualizer::addNeighbor(EGraph::EGraphVertex* v, int neighbor){
  vector<double> coord;
  eg_->discToCont(v,coord);
  visualization_msgs::MarkerArray m = converter_->stateToDetailedVisualizationMarker(coord);
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = m.markers.front().header.frame_id;
  int_marker.pose = m.markers.front().pose;
  int_marker.scale = 1;
  int_marker.description = "";
  int_marker.name = (string("egraph_neighbor_")+boost::lexical_cast<string>(v->id)+string("_")+boost::lexical_cast<string>(neighbor)).c_str();

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

  for(unsigned int i=0; i<m.markers.size(); i++)
    control.markers.push_back(m.markers[i]);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
}

visualization_msgs::MarkerArray EGraphVisualizer::getVisualization(std::string type){
  visualization_msgs::MarkerArray ma;
  
  if(type.compare("egraph") == 0 || type.compare("detailed_egraph") == 0)
  {
    visualization_msgs::MarkerArray m_state;
    for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
      EGraph::EGraphVertex* v = eg_->id2vertex[i];

      bool valid = false;
      for(unsigned int a=0; a<v->valid.size(); a++)
        valid |= v->valid[a];
      if(!valid)
        continue;

      // get state
      vector<double> coord;
      eg_->discToCont(v,coord);

      if(type.compare("detailed_egraph") == 0)
        m_state = converter_->stateToDetailedVisualizationMarker(coord);
      else 
        m_state = converter_->stateToVisualizationMarker(coord);

      // get edges
      for(unsigned int j=0; j<v->neighbors.size(); j++){
        if(!v->valid[j])
          continue;
        EGraph::EGraphVertex* u = v->neighbors[j];
        if(v->id<u->id){
          vector<double> coord2;
          eg_->discToCont(u,coord2);
          visualization_msgs::MarkerArray m = converter_->edgeToVisualizationMarker(coord,coord2);
          ma.markers.insert(ma.markers.end(), m.markers.begin(), m.markers.end());
        }
      }
      ma.markers.insert(ma.markers.end(), m_state.markers.begin(), m_state.markers.end());
    }

    for(unsigned int i = 0; i < ma.markers.size(); ++i)
    {
      ma.markers[i].ns = "egraph";
      ma.markers[i].id = i;
    }
  }
  else
    ROS_ERROR("No visualization of type '%s' is supported.", type.c_str());

  return ma;
}

