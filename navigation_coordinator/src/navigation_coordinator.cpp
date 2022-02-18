#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <service_msg/ServiceMsg.h>
#include <traj_builder/traj_builder.h>

using namespace std;

nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;

ros::ServiceClient client;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

void stop(){
    service_msg::ServiceMsg srv;
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = current_pose;
    srv.request.mode = "0"; 
    if (client.call(srv))
    {
        ROS_INFO("Stopped");
    }
}

bool move2coord(float goal_pose_x, float goal_pose_y)
{
    bool success = true;
    TrajBuilder trajBuilder;
    service_msg::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;

    bool success_rotate;
    bool success_translate;

    // For now: rotate to head forward to goal point, then move toward the place.
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = goal_pose_x;
    double y_end = goal_pose_y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;

    double des_psi = atan2(dy, dx);

    ROS_INFO("Start_x = %f", x_start);
    ROS_INFO("Start_y = %f", y_start);
    ROS_INFO("Goal_x = %f", x_end);
    ROS_INFO("Goal_y = %f", y_end);
    // rotate
    goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                  current_pose.pose.position.y,
                                                  des_psi); // keep the same x,y, only rotate to des_psi
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = goal_pose_rot;
    srv.request.mode = "2"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_rotate = srv.response.success;
        ROS_INFO("rotate success? %d", success_rotate);
    }
    ros::spinOnce();

    // forward
    goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                    goal_pose_y,
                                                    des_psi); // keep des_psi, change x,y
    srv.request.start_pos = goal_pose_rot;
    srv.request.goal_pos = goal_pose_trans;
    srv.request.mode = "1"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_translate = srv.response.success;
        ROS_INFO("translate success? %d", success_translate);
    }
    ros::spinOnce();

    // if fail to forward
    if (!success_translate)
    {
        ROS_INFO("Cannot move, obstacle. braking");
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = current_pose; //anything is fine.
        srv.request.mode = "3";              // spin so that head toward the goal.
        client.call(srv);
        success = false;
    }
    ros::spinOnce();

    return success;
}

void tryMove(float goal_pose_x, float goal_pose_y, int retry_max)
{
    int retry_ctr = 0;
    bool success = move2coord(goal_pose_x, goal_pose_y);
    while (!success && retry_ctr < retry_max) {
        ROS_WARN("RETRY %d", retry_ctr);
        retry_ctr++;
        success = move2coord(goal_pose_x,goal_pose_y);
    }
}

void backUp()
{
    ROS_INFO("Backing up");
    TrajBuilder trajBuilder;
    service_msg::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;

    start_pose.pose = current_state.pose.pose;

    srv.request.start_pos = current_pose;
    srv.request.goal_pos = current_pose;
    srv.request.mode = "4"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        bool success_backup = srv.response.success;
        ROS_INFO("rotate success? %d", success_backup);
    }
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    vector<geometry_msgs::PoseStamped> plan_points;

    client = n.serviceClient<service_msg::ServiceMsg>("des_state_pub");

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    TrajBuilder trajBuilder;

    float x_t1 = 1.0;
    float y_t1 = 0;

    float x_t2 = 1.0;
    float y_t2 = 1.0;
    
    float x_t3 = 0.0;
    float y_t3 = 1.0;  

    float x_o = 0.0;
    float y_o = 0.0;

    ROS_INFO("STEP 1: Coordinate 1");
    tryMove(x_t1, y_t1, 1);

    ROS_INFO("STEP 2: Coordinate 2");
    tryMove(x_t2, y_t2, 1);

    ROS_INFO("Step 3: Coordinate 3");
    tryMove(x_t3, y_t3, 1);

    ROS_INFO("Step 4: Home");
    tryMove(x_o,y_o, 1);


    ROS_INFO("Shutting down motor");
    // stop everything
    stop();

    ros::spin();

    return 0;
}
