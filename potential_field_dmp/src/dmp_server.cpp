#include "dmp_server.h"
#include "dmp.h"
#include "potential_field_dmp/Trajectory_1D.h"
#include "potential_field_dmp/Parameters.h"
#include "potential_field_dmp/Plan.h"
#include "potential_field_dmp/WayPoint.h"
#include "potential_field_dmp/SetParamDMP.h"
#include "potential_field_dmp/GenerateDMPPlan.h"
#include "iostream"
#include "ros/ros.h"

using namespace std;
using namespace potential_field_dmp;

std::vector<DMPequation*> learned_dmp;

// calls GenerateTrajectory_nD function
// returns the plan generated from the previously learned linear DMP
// and the given start, goal, initial_vel, dt, and tau values
bool GenerateDMPPlanHandler(GenerateDMPPlan::Request  &req,
                            GenerateDMPPlan::Response &res ) {
  std::cout << "Request received for planning" << std::endl;
  if (learned_dmp.size() != req.start.size())
    return false;

  GenerateTrajectory_nD(req.start,
                        req.goal,
                        req.initial_velocity,
                        req.obs_pos,
                        req.dt,
                        req.tau,
                        learned_dmp,
                        res.generated_plan);
  return true;
}

// calls SetParametersDMP_nD function
// updates the learned_dmp vector to contain
// the set of n learned DMP equations
bool SetParametersDMPHandler(SetParamDMP::Request  &req,
                             SetParamDMP::Response &res ) {
  std::cout << "Request received for setting parameters" << std::endl;
  // for (int j = 0; j < req.demo.traj[0].waypoint.size(); ++j)
  // {
  //     for (int i = 0; i < 3; ++i)
  //     {
  //         std::cout << req.demo.traj[i].waypoint[j];
  //     }
  //     std::cout<<"------------"<<std::endl;
  // }

  SetParametersDMP_nD(req.param, learned_dmp);
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dmp_server");
  ros::NodeHandle n;

  ros::ServiceServer set_dmp = n.advertiseService("set_dmp", SetParametersDMPHandler);
  ros::ServiceServer generate_dmp_plan = n.advertiseService("generate_dmp_plan", GenerateDMPPlanHandler);

  ros::spin();
  return 0;
}