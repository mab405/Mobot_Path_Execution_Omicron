// wsn example program to illustrate LIDAR processing.  1/23/15
// modified by mct60 S'22 for ECSE 476 PS4 lidar_alarm node -Team Omicron


// Node is an upgrade of the STDR LIDAR alarm which subscribes to the mobot’s LIDARtopic 
// and interpret safe paths towards a goal pose with sufficient look-ahead for graceful braking.

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>

#define PI M_PI

const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
//int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool laser_initialized = false;

//assumption of counterclockwise indexing
double window_deg = 10.0;
double angle_min_alarm = (window_deg/2)*(-PI/180); //setting angle boundary on left side
double angle_max_alarm = (window_deg/2)*(PI/180); //setting angle boundary on right side
double range_min_alarm = 0.2; //minimum distance of interest
double range_max_alarm = 0.6; //maximum distance of interest
// triggers when distance is greater than 2 meters
int max_ping_index = 0;//Index of most right angle
int min_ping_index = 0;//Index of most left angle

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (!laser_initialized)  {
        //when first message is received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

        
        //set ping indices to correspond to angles of interest
        max_ping_index = (int) ((angle_max_alarm - angle_min_)/angle_increment_);
        min_ping_index = (int) ((angle_min_alarm - angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup");
        
        laser_initialized = true;
    }
    
    laser_alarm_=false;

    //scans lidar data within ping indices to find ranges
    for (int ping_index_ = min_ping_index; ping_index_ <= max_ping_index; ping_index_++){
    	ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   		//ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   	
   		//checks if each lidar data point is within alarm distance
   		if (ping_dist_in_front_<range_max_alarm && ping_dist_in_front_ > range_min_alarm) {
       		laser_alarm_=true;
   		}	
    }

    //sends warning if data point trips lidar alarm
    if (laser_alarm_){
      ROS_WARN("LIDAR ALARM TRIGGERED! POTENTIAL OBSTACLE DETECTED!");
    }

   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}
