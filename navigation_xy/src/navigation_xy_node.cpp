#include <navigation_xy/navigation_xy_node.h>
#include <nav_msgs/Path.h>

using namespace std;

EGraphXYNode::EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  env_ = new EGraphXY(costmap_ros_->getResolution());

  env_->InitializeEnv(costmap_ros_->getSizeInCellsX(),//width
                      costmap_ros_->getSizeInCellsY(),//height
                      NULL, //map
                      0,0, //start
                      0,0, //goal
                      costmap_2d::INSCRIBED_INFLATED_OBSTACLE); //obs thresh

  for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, cost_map_.getCost(ix,iy));

  string egraph_filename;
  private_nh.param<string>("egraph_filename", egraph_filename, "");
  if(egraph_filename.empty())
    egraph_ = new EGraph(env_,2, 0);
  else
    egraph_ = new EGraph(env_,egraph_filename);

  heur_ = new EGraphEuclideanHeuristic(*env_,ENVNAV2D_COSTMULT);
  egraph_mgr_ = new EGraphManager<vector<double> >(egraph_, env_, heur_);
  planner_ = new LazyAEGPlanner<vector<double> >(env_, true, egraph_mgr_);
  egraph_vis_ = new EGraphVisualizer(egraph_, env_);

  egraph_vis_->visualize();

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback,this);
  plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
  plan_service_ = nh.advertiseService("/sbpl_planning/plan_path",&EGraphXYNode::makePlan,this);
}

void EGraphXYNode::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  planner_->interrupt();
}

bool EGraphXYNode::makePlan(navigation_xy::GetXYPlan::Request& req, navigation_xy::GetXYPlan::Response& res){
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  unsigned int sx,sy,gx,gy;
  cost_map_.worldToMap(req.start_x,req.start_y,sx,sy);
  cost_map_.worldToMap(req.goal_x,req.goal_y,gx,gy);

  int ret = env_->SetStart(sx,sy);
  if(ret < 0 || planner_->set_start(ret) == 0){
    ROS_ERROR("ERROR: failed to set start state\n");
    return false;
  }
  ret = env_->SetGoal(gx,gy);
  if(ret < 0 || planner_->set_goal(ret) == 0){
    ROS_ERROR("ERROR: failed to set goal state\n");
    return false;
  }

  for(unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++)
    for(unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++)
      env_->UpdateCost(ix, iy, cost_map_.getCost(ix,iy));

  EGraphReplanParams params(5.0);
  params.initial_eps = req.initial_eps;
  params.dec_eps = req.dec_eps;
  params.final_eps = req.final_eps;
  params.epsE = req.egraph_eps;
  params.dec_epsE = req.dec_egraph_eps;
  params.final_epsE = req.final_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = req.use_egraph;
  params.feedback_path = req.feedback_path;

  vector<int> solution_stateIDs;
  ret = planner_->replan(&solution_stateIDs, params);

  map<string,double> stats = planner_->getStats();
  for(map<string,double>::iterator it=stats.begin(); it!=stats.end(); it++){
    res.stat_names.push_back(it->first);
    res.stat_values.push_back(it->second);
  }
  if(!ret)
    return false;

  if(req.save_egraph)
    egraph_->save("xy_egraph.eg");

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(solution_stateIDs.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();
  for(unsigned int i=0; i<solution_stateIDs.size(); i++){
    int x,y;
    env_->GetCoordFromState(solution_stateIDs[i],x,y);
    double wx,wy;
    cost_map_.mapToWorld(x,y,wx,wy);

    gui_path.poses[i].pose.position.x = wx;
    gui_path.poses[i].pose.position.y = wy;

    geometry_msgs::PoseStamped p;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = 0;

    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    res.path.push_back(p);
  }
  plan_pub_.publish(gui_path);

  egraph_vis_->visualize();

  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_xy_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  EGraphXYNode xy(costmap);

  //ros::spin();
  ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch a the interrupt
  spinner.spin();
}
