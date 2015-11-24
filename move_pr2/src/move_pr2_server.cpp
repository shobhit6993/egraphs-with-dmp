#include "move_pr2/move_pr2_server.h"
#include "move_pr2/MovePR2.h"
#include "iostream"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf/transform_listener.h>
// #include <gazebo_msgs/LinkState.h>

using namespace std;
using namespace move_pr2;

RobotDriver robot_driver;

double CalcDistance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
    double x_offset = p1.pose.position.x - p2.pose.position.x;
    double y_offset = p1.pose.position.y - p2.pose.position.y;
    return sqrt(x_offset * x_offset + y_offset * y_offset);
}

void RobotDriver::set_cmd_vel_pub(ros::NodeHandle& n) {
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("base_controller/command", 5);
    // set_link_state_pub_ = nh_.advertise<gazebo_msgs::LinkState>("gazebo/set_link_state", 5);

}

void RobotDriver::Move(const MovePR2::Request &req) {
    tf::TransformListener listener;
    int n = req.path.size();
    listener.waitForTransform("base_footprint", "map",
                              ros::Time(0), ros::Duration(1000.0));

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    geometry_msgs::Twist base_cmd;
    ros::Rate rate(10.0);
    double dist_moved, distance;
    for (int i = 0; i < n ; i++) {
        base_cmd = req.vel[i];
        if (i == n - 1) {
            cmd_vel_pub_.publish(base_cmd);
            break;
        }
        else
            distance = CalcDistance(req.path[i], req.path[i + 1]);

        std::cout << "i=" << i << "dist=" << distance << std::endl;
        //record the starting transform from the odometry to the base frame
        listener.lookupTransform("base_footprint", "map",
                                 ros::Time(0), start_transform);

        dist_moved = 0;
        while (dist_moved <= distance - kTolerance) {
            cmd_vel_pub_.publish(base_cmd);
            rate.sleep();
            try
            {
                listener.lookupTransform("base_footprint", "map",
                                         ros::Time(0), current_transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                break;
            }
            //see how far we've traveled
            tf::Transform relative_transform =
                start_transform.inverse() * current_transform;
            dist_moved = relative_transform.getOrigin().length();
            std::cout << "dist moved=" << dist_moved << std::endl;
        }
    }
}

bool MovePR2Handler(MovePR2::Request  &req,
                    MovePR2::Response &res ) {
    std::cout << "Request received for moving PR2" << std::endl;
    robot_driver.Move(req);
    std::cout << "Request servicing complete" << std::endl;
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "move_pr2_server");
    ros::NodeHandle n;

    robot_driver.set_cmd_vel_pub(n);

    ros::ServiceServer srv = n.advertiseService("move_pr2_service", MovePR2Handler);

    ros::spin();
    return 0;
}