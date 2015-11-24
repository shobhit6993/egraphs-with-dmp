#ifndef INTERACT_XY_H
#define INTERACT_XY_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <navigation_xy/GetXYPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class ControlPlanner {
public:
    void callPlanner();
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void PublishDMPPlan(const std::vector<geometry_msgs::PoseStamped> &dmp_pose);
    void MoveRobot(const navigation_xy::GetXYPlan::Response &r);
    void ClearReqRes();
    void MoveObstacle();
    ControlPlanner();

private:
    interactive_markers::InteractiveMarkerServer* server;
    interactive_markers::MenuHandler menu_handler;
    costmap_2d::Costmap2DROS* planner_costmap_ros_;
    FILE* fout;
    tf::TransformListener tf_;
    boost::thread* planner_thread;
    boost::mutex mutex;
    boost::condition_variable call_planner_cond;
    ros::ServiceClient planner;
    ros::Publisher interrupt_pub;
    ros::Publisher plan_pub_;
    navigation_xy::GetXYPlan::Request req;
    navigation_xy::GetXYPlan::Response res;
    int test_num;
};

#endif
