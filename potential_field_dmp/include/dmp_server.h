#ifndef DMP_SERVER_H
#define DMP_SERVER_H

#include "ros/ros.h"
#include "potential_field_dmp/SetParamDMP.h"
#include "potential_field_dmp/GenerateDMPPlan.h"

using namespace std;
using namespace potential_field_dmp;


bool GenerateDMPPlanHandler(GenerateDMPPlan::Request  &req,
                            GenerateDMPPlan::Response &res );

bool SetParametersDMPHandler(SetParamDMP::Request  &req,
                             SetParamDMP::Response &res );

#endif //DMP_SERVER_H
