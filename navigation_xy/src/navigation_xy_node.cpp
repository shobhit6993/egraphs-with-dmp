#include <navigation_xy/navigation_xy_node.h>
#include <nav_msgs/Path.h>
#include <navigation_xy/constants.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <math.h>

using namespace std;
using namespace potential_field_dmp;

int m_id = 0; //marker_id

EGraphXYNode::EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros) {
  // ros::NodeHandle nh_;
  ros::NodeHandle private_nh("~");

  if (!SetParametersDMP())
    return;

  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  env_ = new EGraphXY(costmap_ros_->getResolution());

  env_->InitializeEnv(costmap_ros_->getSizeInCellsX(),//width
                      costmap_ros_->getSizeInCellsY(),//height
                      NULL, //map
                      0, 0, //start
                      0, 0, //goal
                      costmap_2d::INSCRIBED_INFLATED_OBSTACLE); //obs thresh

  for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, cost_map_.getCost(ix, iy));

  string egraph_filename;
  private_nh.param<string>("egraph_filename", egraph_filename, "");
  if (egraph_filename.empty())
    egraph_ = new EGraph(env_, 2, 0);
  else
    egraph_ = new EGraph(env_, egraph_filename);

  heur_ = new EGraphEuclideanHeuristic(*env_, ENVNAV2D_COSTMULT);
  egraph_mgr_ = new EGraphManager<vector<double> >(egraph_, env_, heur_);
  planner_ = new LazyAEGPlanner<vector<double> >(env_, true, egraph_mgr_);
  egraph_vis_ = new EGraphVisualizer(egraph_, env_);

  egraph_vis_->visualize();

  interrupt_sub_ = nh_.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback, this);
  plan_pub_ = nh_.advertise<nav_msgs::Path>("plan_without_dmp", 1);
  plan_service_ = nh_.advertiseService("/sbpl_planning/plan_path", &EGraphXYNode::makePlan, this);
}

void EGraphXYNode::interruptPlannerCallback(std_msgs::EmptyConstPtr) {
  ROS_WARN("Planner interrupt received!");
  planner_->interrupt();
}

bool EGraphXYNode::makePlan(navigation_xy::GetXYPlan::Request& req, navigation_xy::GetXYPlan::Response& res) {
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  unsigned int sx, sy, gx, gy;
  cost_map_.worldToMap(req.start_x, req.start_y, sx, sy);
  cost_map_.worldToMap(req.goal_x, req.goal_y, gx, gy);

  int ret = env_->SetStart(sx, sy);
  if (ret < 0 || planner_->set_start(ret) == 0) {
    ROS_ERROR("ERROR: failed to set start state\n");
    return false;
  }
  ret = env_->SetGoal(gx, gy);
  if (ret < 0 || planner_->set_goal(ret) == 0) {
    ROS_ERROR("ERROR: failed to set goal state\n");
    return false;
  }

  for (unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++)
    for (unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++)
      env_->UpdateCost(ix, iy, cost_map_.getCost(ix, iy));

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

  map<string, double> stats = planner_->getStats();
  for (map<string, double>::iterator it = stats.begin(); it != stats.end(); it++) {
    res.stat_names.push_back(it->first);
    res.stat_values.push_back(it->second);
  }
  if (!ret)
    return false;

  if (req.save_egraph)
    egraph_->save("xy_egraph.eg");

  //create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(solution_stateIDs.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < solution_stateIDs.size(); i++) {
    int x, y;
    env_->GetCoordFromState(solution_stateIDs[i], x, y);
    double wx, wy;
    cost_map_.mapToWorld(x, y, wx, wy);

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

  if (!HandleMovingObstacles(req, res))
    return false;

  plan_pub_.publish(gui_path);  // publishes the generated plan (without DMP)
  egraph_vis_->visualize();

  return true;
}

void EGraphXYNode::SetVelocity(int i, navigation_xy::GetXYPlan::Response& res) {
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  double x = res.path[i + 1].pose.position.x - res.path[i].pose.position.x;
  double y = res.path[i + 1].pose.position.y - res.path[i].pose.position.y;

  double sign_x = (x >= 0) ? 1 : -1;
  double sign_y = (y >= 0) ? 1 : -1;

  double angle = abs(atan(y / x));
  vel.linear.x = sign_x * kSpeed * cos(angle);
  vel.linear.y = sign_y * kSpeed * sin(angle);
  res.vel.push_back(vel);
}

double CalcDistance(int s, int e, const navigation_xy::GetXYPlan::Response& res) {

  double x = res.path[e].pose.position.x - res.path[s].pose.position.x;
  double y = res.path[e].pose.position.y - res.path[s].pose.position.y;
  return sqrt(x * x + y * y);
}

void UpdateSE(const navigation_xy::GetXYPlan::Response& res,
              double target_dist,
              int& s,
              int& e,
              std::vector <geometry_msgs::PoseStamped> corrected_path) {
  int n = res.path.size();
  int i = e;
  double dist_travelled = CalcDistance(i, i + 1, res);
  corrected_path.push_back(res.path[i]);
  i++;

  while (i < n - 1 && dist_travelled <= target_dist) {
    dist_travelled = dist_travelled + CalcDistance(i, i + 1, res);
    corrected_path.push_back(res.path[i]);
    i++;
  }

  s = e + 1;
  e = i;
}

void CalcObstaclePos(const navigation_xy::GetXYPlan::Request& req,
                     double& obs_curr_x,
                     double& obs_curr_y) {
  obs_curr_x = obs_curr_x + kObsSpeed * (kSleep / 1000000.0);
  obs_curr_x = obs_curr_y + kObsSpeed * (kSleep / 1000000.0);
}

void EGraphXYNode::PlotPoint(geometry_msgs::PoseStamped p) {
  // std::cout << m_id << std::endl;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;

  // geometry_msgs::PoseStamped w = ar_m.pose;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  // marker.header.stamp = ar_m.header.stamp;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "dynamic_marker";
  marker.id = m_id;
  m_id++;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose = p.pose;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.b = 1.0f;
  marker.color.g = 0.0f;
  marker.color.r = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  ros::Rate loop_rate(30);
  // while (ros::ok())
  // {
  marker_pub_.publish(marker);
  loop_rate.sleep();
  // }
}

void EGraphXYNode::PlotPoints(int s, int e, const navigation_xy::GetXYPlan::Response & res) {
  for (int i = s; i <= e; ++i) {
    PlotPoint(res.path[i]);
  }
}

// only for one obstacle
bool EGraphXYNode::HandleMovingObstacles(const navigation_xy::GetXYPlan::Request & req,
    navigation_xy::GetXYPlan::Response & res) {
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("path_marker", 1);

  int num_points = res.path.size();
  int dmp_start_offset, dmp_goal_offset;
  geometry_msgs::PoseStamped dmp_start, dmp_goal, new_point;
  std::vector <geometry_msgs::PoseStamped> corrected_path;

  int s = 0, e = 0;
  PlotPoint(res.path[0]);

  double target_dist = kRobotSpeed * (kSleep / 1000000.0);
  double obs_curr_x = req.obs_x[0];
  double obs_curr_y = req.obs_y[0];
  while (e < num_points - 1) {
    usleep(kSleep);
    UpdateSE(res, target_dist, s, e, corrected_path);
    // std::cout << "s=" << s << " e=" << e << std::endl;
    PlotPoints(s, e, res);
    CalcObstaclePos(req, obs_curr_x, obs_curr_y);
    if (IsInCollision(res.path[e], obs_curr_x, obs_curr_y, req.base_radius)) {
      std::cout << "e=" << e << " " << res.path[e].pose.position.x
                << " " << res.path[e].pose.position.y
                << "..." << obs_curr_x << " " << obs_curr_y << std::endl;

      dmp_start_offset = e;
      dmp_start = res.path[dmp_start_offset];

      dmp_goal_offset = (e + kOffset < num_points) ? e + kOffset : num_points - 1;
      dmp_goal = res.path[dmp_goal_offset];
      e = dmp_goal_offset;

      Plan dmp_plan;
      geometry_msgs::Twist vel;
      vel.linear.x = kRobotSpeed;
      vel.linear.y = kRobotSpeed;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = 0;
      if (!GenerateDMPPlan(dmp_start, dmp_goal, vel, obs_curr_x, obs_curr_y, dmp_plan))
        return false;

      for (int w = 0; w < dmp_plan.traj[0].waypoint.size(); ++w) {  // for each waypoint
        new_point.pose.position.x = dmp_plan.traj[0].waypoint[w].position;
        new_point.pose.position.y = dmp_plan.traj[1].waypoint[w].position;
        PlotPoint(new_point);
        corrected_path.push_back(new_point);
      }
    }
  }
  res.path = corrected_path;
  return true;
}

bool EGraphXYNode::HandleOnlineObstacles(const navigation_xy::GetXYPlan::Request & req,
    navigation_xy::GetXYPlan::Response & res) {
  int num_obstacles = req.obs_x.size();
  int num_points = res.path.size();

  int dmp_start_offset, dmp_goal_offset;
  geometry_msgs::PoseStamped dmp_start, dmp_goal, new_point;
  std::vector <geometry_msgs::PoseStamped> corrected_path;

  new_point.pose.position.z = 0;
  new_point.pose.orientation.w = 1;
  new_point.pose.orientation.x = 0;
  new_point.pose.orientation.y = 0;
  new_point.pose.orientation.z = 0;

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  bool collision = false;
  int i = 0;
  int v_idx = -1;
  while (i < num_points) {

    if (i != num_points - 1) {
      SetVelocity(i, res);
      v_idx++;
    } else {
      vel.linear.x = 0;
      vel.linear.y = 0;
      res.vel.push_back(vel);
      v_idx++;
    }

    collision = false;
    for (int o = 0; o < num_obstacles; ++o) {
      if (IsInCollision(res.path[i], req.obs_x[o], req.obs_y[o], req.base_radius)) {
        std::cout << "i=" << i << " " << res.path[i].pose.position.x
                  << " " << res.path[i].pose.position.y
                  << "..." << req.obs_x[o] << " " << req.obs_y[o] << std::endl;

        dmp_start_offset = i;
        dmp_start = res.path[dmp_start_offset];

        // search for first point on egraphs path that is beyond obstacles' influence
        i++;
        while (i < num_points) {
          if (!IsInCollision(res.path[i], req.obs_x[o], req.obs_y[o], req.base_radius))
            break;

          i++;
        }

        dmp_goal_offset = i;
        dmp_goal = res.path[dmp_goal_offset];

        Plan dmp_plan;
        if (!GenerateDMPPlan(dmp_start, dmp_goal, res.vel[v_idx], req.obs_x[o], req.obs_y[o], dmp_plan))
          return false;

        for (int w = 0; w < dmp_plan.traj[0].waypoint.size(); ++w) {  // for each waypoint
          new_point.pose.position.x = dmp_plan.traj[0].waypoint[w].position;
          new_point.pose.position.y = dmp_plan.traj[1].waypoint[w].position;
          corrected_path.push_back(new_point);

          vel.linear.x = dmp_plan.traj[0].waypoint[w].velocity;
          vel.linear.y = dmp_plan.traj[1].waypoint[w].velocity;
          res.vel.push_back(vel);
          v_idx++;
        }

        collision = true;
        i = dmp_goal_offset + 1;
      }
    }
    if (!collision) {
      corrected_path.push_back(res.path[i]);
      i++;
    }
  }

  res.path = corrected_path;
  return true;
}

bool EGraphXYNode::IsInCollision(const geometry_msgs::PoseStamped & point,
                                 double dmp_obs_x,
                                 double dmp_obs_y,
                                 double base_radius) {
  double x_clearance = abs(point.pose.position.x - dmp_obs_x);
  double y_clearance = abs(point.pose.position.y - dmp_obs_y);
  double dist = sqrt(x_clearance * x_clearance + y_clearance * y_clearance);
  return (dist < 1.2 * base_radius);
}

bool EGraphXYNode::GenerateDMPPlan(const geometry_msgs::PoseStamped & dmp_start,
                                   const geometry_msgs::PoseStamped & dmp_goal,
                                   geometry_msgs::Twist initial_vel,
                                   double dmp_obs_x,
                                   double dmp_obs_y,
                                   Plan & dmp_plan) {

  ros::ServiceClient gen_dmp_plan_client =
    nh_.serviceClient<potential_field_dmp::GenerateDMPPlan>("generate_dmp_plan");
  potential_field_dmp::GenerateDMPPlan gen_plan_srv;

  std::vector<double> start(kDOF);
  std::vector<double> goal(kDOF);
  std::vector<double> obs_pos(kDOF);
  std::vector<double> initial_velocity(kDOF);

  start[0] = dmp_start.pose.position.x;
  start[1] = dmp_start.pose.position.y;

  goal[0] = dmp_goal.pose.position.x;
  goal[1] = dmp_goal.pose.position.y;

  obs_pos[0] = dmp_obs_x;
  obs_pos[1] = dmp_obs_y;

  initial_velocity[0] = initial_vel.linear.x;
  initial_velocity[1] = initial_vel.linear.y;

  gen_plan_srv.request.start = start;
  gen_plan_srv.request.goal = goal;
  gen_plan_srv.request.obs_pos = obs_pos;
  gen_plan_srv.request.initial_velocity = initial_velocity;
  gen_plan_srv.request.tau = kParamTau;
  gen_plan_srv.request.dt = kParamDT;
  gen_plan_srv.request.mode = dmp_mode_;

  if (gen_dmp_plan_client.call(gen_plan_srv)) {
    dmp_plan = gen_plan_srv.response.generated_plan;
    return true;
  }
  else {
    ROS_ERROR("Failed to call service generate_dmp_plan");
    return false;
  }
}

bool EGraphXYNode::SetParametersDMP() {
  ROS_INFO("Waiting for set_dmp service to come up");
  ros::service::waitForService("set_dmp", -1);
  ROS_INFO("set_dmp service alive");

  ros::ServiceClient set_param_dmp_client =
    nh_.serviceClient<potential_field_dmp::SetParamDMP>("set_dmp");
  potential_field_dmp::SetParamDMP set_dmp_srv;

  param_.resize(kDOF);
  if (!ReadParameters())
    return false;

  set_dmp_srv.request.param = param_;

  if (set_param_dmp_client.call(set_dmp_srv)) {
    return true;
  } else {
    ROS_ERROR("Failed to call service set_dmp");
    return false;
  }
}

bool EGraphXYNode::ReadParameters() {
  ifstream fin;
  fin.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

  try {
    fin.open(kParametersFile);
    for (int i = 0; i < kDOF; ++i) {
      fin >> param_[i].k;
      fin >> param_[i].d;
      fin >> param_[i].tau;
      fin >> param_[i].eta;
      fin >> param_[i].p_0;
    }
    fin >> dmp_mode_;
    fin.close();
    return true;
  } catch (std::ifstream::failure e) {
    std::cerr << e.what() << endl;
    if (fin.is_open()) fin.close();
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_xy_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  EGraphXYNode xy(costmap);

  //ros::spin();
  ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch a the interrupt
  spinner.spin();
}
