#include "ros/ros.h"
#include "move_pr2/MovePR2.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
using namespace move_pr2;

const double kTolerance = 0.009;

bool MovePR2Handler(MovePR2::Request  &req,
                    MovePR2::Response &res );

double CalcDistance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);

class RobotDriver {
private:
    ros::Publisher cmd_vel_pub_;

public:
    void set_cmd_vel_pub(ros::NodeHandle &nh);
    void Move(const MovePR2::Request  &req);
};