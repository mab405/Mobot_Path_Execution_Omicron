# Mobot_Path_Execution_Omicron
  - Problem Set 4 Assignment for Mobile Robotics
  - Video Link: https://drive.google.com/file/d/1Fwj9bjyXDikaGAojXXvVX1_eXt8gf8tR/view?usp=sharing
  
## Reference Code
   - Code from Professor Newman's Repo was used as refernce
     - https://github.com/wsnewman/learning_ros/tree/master/Part_4
   - For clarity we named our Assignment nodes to be consistent with the HW pdf
     - Our Package Variation = Equivalent From Reference Code
## Nodes
1. current_state_publisher = odom_tf
   - This node will later combine absolute pose information (e.g. from GPS or LIDAR/map-based localization) with high-speed Odom information. For this assignment, this node should merely subscribe to the Odom topic and republish Odom on the topic “current_state”.
   - Essentially the alterations we made were adding a new topic for odom information to be subscribed to 'current_state'
2. lidar_alarm = lidar_alarm_omicron (based on Part 2 Code)
   - Part 2 Code Reference: https://github.com/wsnewman/learning_ros/tree/master/Part_2/lidar_alarm
   - This node should be an upgrade of the STDR LIDAR alarm. It should subscribe to the mobot’s LIDAR topic and interpret safe paths towards a goal pose with sufficient look-ahead for graceful braking.
   -  Basically it just has a TRUE/FALSE lidar_alarm and when the lidar_alarm senses something within a given range (that we arbitrarily choose) it reads as TRUE
3. modal_trajectory_controller = lin_steering
   - This node will be upgraded to perform “lane-drift” correction, heading control and path progress control. For this assignment, implement this equivalent to the open-loop controller, which merely copies a desired state twist to cmd_vel. The controller should be modified to prepare for different control modes: spin-in-place, straight-line-motion, and halt.
   - As mentioned above this funciton 'Steers' the robot
   - Additional functionality was added in the midterm assignment but for this one is simply creates a 're-publisher' that republishes the des_state information to control the mobot
4. des_state_publisher_service = des_pub_state
   - This node should use functions from the traj_builder library (Part4/traj_builder) to construct triangular and trapezoidal trajectory plans for either forward travel or spin-in-place motions. It should receive a goal pose as a service request. It should attempt to stream sequential desired states in accordance with the request, resulting in returning either success or failure. Reasons for failure would include: encountering a lidar_alarm prior to reaching the goal pose, and failure to converge on the goal pose within some tolerance. In response to a lidar_alarm, this service should dynamically construct and publish (stream) a graceful braking trajectory.
   - ADDED IN CASES FOR: 
     - FORWARD
     - STOP
     - SPIN
   - When invoked these 'cases' cause the robot to perform the desired pre-planned action for control
   - In essence the 'desired state' becomes what action you want the mobot to perform
5. traj_builder = traj_builder
   - This node should contain a plan for a sequence of path vertices, and send these as requests one at a time to the des_state_publisher_service. The coordinator will suspend while waiting for a response from the des_state_publisher_service. If the response is “false”, the coordinator should pause briefly, then resend the last, unsuccessful goal vertex. (This could be adequate if a pedestrian blocks the robot, then subsequently walks away). The navigation_coordinator will grow in sophistication to incorporate path planning and path replanning (e.g. to circumvent unexpected obstacles).
   - Creates a series of desired points for the robot to travel to using the above controller options
## Structure
1. For Conveience was structured into a series of 'catkin_create_pkg' package files
   - Conveient for future assignments
2. Each file contains:
   - src
     - code
     - test code
   - README.md w/ basic information
   - CMAKE
   - Project.xml
3. des_state_publisher_service & navigation_coordinator work in tandem
   - all others are independent
## IMPORTANT NOTE
   - NOETIC
     - CMAKE files use <run_depend></run_depend>
   - MELODIC
     - CMAKE files use <exec_depend></exec_depend>

## Discussion
   - Had difficulties in making mobot_urdf and given files for mobot alongside our altered files to try
     - catkin_make --help gave us some useful options in fixing these difficulties
     - Particularly, the 'make only packages with dependencies' option for our packages and Prof Newman's Part 2 mobot_urdf package
   - In our updated 'midterm' assignment we also added in a launch file to run of all of the nodes except the navigation_coodinator for convenience in testing
   - Lidar alarm was having issues in testing - will need to test in midterm assignment when code is up and running

