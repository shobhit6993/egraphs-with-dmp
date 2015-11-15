#ifndef DMP_SERVER_H
#define DMP_SERVER_H

#include "ros/ros.h"
#include "potential_field_dmp/TrainLinearDMP.h"
#include "potential_field_dmp/GenerateLinearDMPPlan.h"

using namespace std;
using namespace potential_field_dmp;


bool GenerateLinearDMPPlanHandler(GenerateLinearDMPPlan::Request  &req,
                                  GenerateLinearDMPPlan::Response &res );

bool SetParametersDMPHandler(TrainLinearDMP::Request  &req,
                             TrainLinearDMP::Response &res );

#endif //DMP_SERVER_H
