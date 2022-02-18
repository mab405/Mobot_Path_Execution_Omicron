//odom_tf.cpp:
//wsn, March 2016
//illustrative node to show use of tf listener to reconcile odom and map frames
// w/rt base_link

// this header incorporates all the necessary #include files and defines the class "OdomTf"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <current_state_publisher/odom_tf.h>

using namespace std;



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "OdomTf_node"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
	ros::Publisher pub = nh.advertise<std_msgs::String>("OdomTf_node", 1);
    ros::Publisher pub2 = nh.advertise<std_msgs::String>("OdomTf_node",1);

    ROS_INFO("main: instantiating an object of type OdomTf");
    OdomTf odomTf(&nh);  //instantiate an OdomTf object and pass in pointer to nodehandle for constructor to use
    OdomTf current_state_subscriber(&nh);

    ROS_INFO:("starting main loop");
    ros::Rate sleep_timer(50.0);
    while (ros::ok()) {
        ros::spinOnce();
        sleep_timer.sleep(); 
    }
    
    return 0;
} 
