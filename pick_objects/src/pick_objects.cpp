#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void notifyState(ros::Publisher& publisher, const int& state) {
  //Created a Int32 message
  std_msgs::Int32 msg;

  //Inserted data to message header
  msg.data = state;

  //Printing message data
  ROS_INFO_STREAM("state: "<< msg.data);
  //Publishing the topic 
  publisher.publish(msg);
}

void spinState(ros::Rate& r, 
          ros::Publisher& publisher, 
          const int& state, 
          const int& steps) {
 int spin_step = 0;
  while (spin_step <= steps) {
  	notifyState(publisher, state);
    //Spinning once for doing the  all operation once
	ros::spinOnce();
    r.sleep();
    ++spin_step;
  }
}

void moveToLocation(MoveBaseClient& ac,
                    ros::Publisher& publisher,
                	const float& x, 
                	const float& y, 
                	const float& w, 
                	const std::string& msg, 
                    const int& state) {
  
  move_base_msgs::MoveBaseGoal goal;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO_STREAM("Moving to: " << msg
                 << " x: " << goal.target_pose.pose.position.x 
                 << " y: " << goal.target_pose.pose.position.y 
                 << " w: " << goal.target_pose.pose.orientation.w);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Robot has reached the: " << msg 
                    << " x: " << goal.target_pose.pose.position.x 
                    << " y: " << goal.target_pose.pose.position.y 
                    << " w: " << goal.target_pose.pose.orientation.w);
    notifyState(publisher, state);
  } else {
    ROS_INFO_STREAM("Robot has failed to reach the: " << msg);
  }
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Publisher
  ros::Publisher pick_object_publisher = 
    n.advertise<std_msgs::Int32>("/pick_object",10);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::Rate r(1);
  spinState(r, pick_object_publisher, 0, 1);
  
  // Pickup area
  moveToLocation(ac, pick_object_publisher, 2.97, 6.21, 2.1, "pickup zone", 1);
  sleep(5);
  // Drop off area
  moveToLocation(ac, pick_object_publisher, -3.1, 6.01, -2.28, "drop off zone", 2);
  ros::spin();
  return 0;
}