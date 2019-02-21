#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

int curr_state = -1;

visualization_msgs::Marker getMarker(const int& action, 
                                     const int& x, 
                                     const int& y) {
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
 	return marker;   
}

//Callback of the topic /numbers
void pick_object_callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO_STREAM("Recieved  state: " <<  msg->data);
  curr_state = msg->data;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker_base");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pick_object_subscriber = n.subscribe("/pick_object",10, pick_object_callback);
  // TODO(saminda)
  std::vector<visualization_msgs::Marker> markers;
  while (ros::ok()) {
    visualization_msgs::Marker marker;
    if (curr_state == 0) {
      	markers.push_back(getMarker(visualization_msgs::Marker::ADD, 3.0, 6.5));
        ROS_INFO_STREAM("ADD pickup zone");
    } else if (curr_state == 1) {
      	markers.push_back(getMarker(visualization_msgs::Marker::DELETE, 3.0, 6.5));
      	markers.push_back(getMarker(visualization_msgs::Marker::ADD, -3.2, 6.5));
        ROS_INFO_STREAM("DELETE pickup zone");
        ROS_INFO_STREAM("ADD drop off zone");
    } else if (curr_state == 2) {
        markers.push_back(getMarker(visualization_msgs::Marker::DELETE, -3.2, 6.5));
        ROS_INFO_STREAM("DELETE drop off zone");
    }
    curr_state = -1;
  	// Publish the marker
    if (marker_pub.getNumSubscribers() > 0) {
    	for (size_t i = 0; i < markers.size(); ++i) {
       		marker_pub.publish(markers[i]);
        }
    } else {
    	ROS_WARN_ONCE("Please create a subscriber to the marker");
    }
    ros::spinOnce();
    r.sleep();
  }
  ros::spin();
  return 0;
}