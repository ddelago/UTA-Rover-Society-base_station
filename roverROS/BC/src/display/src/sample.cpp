#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Int8.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "sample");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker3", 1);
	ros::Publisher vol_pub = n.advertise<std_msgs::Int8>("datavol_1", 2);

	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;
	ros::Publisher pub = n.advertise<std_msgs::Int8>("topic_name", 5);
	while (ros::ok())
	{

		std_msgs::Int8 str;
		str.data = 24;
		pub.publish(str);

		visualization_msgs::Marker marker;

		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = 0;


		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.text = "11";
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();
		std_msgs::Int8 msg;
		msg.data = 19;
		// Publish the marker
		while ( marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the markers!");
			sleep(0.1);
		}
		marker_pub.publish(marker);
		vol_pub.publish(msg);
		//	marker_pub.publish(temp);
		// Cycle between different shapes
		switch (shape)
		{
		case visualization_msgs::Marker::CUBE:
			shape = visualization_msgs::Marker::SPHERE;
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;

			break;
		case visualization_msgs::Marker::SPHERE:
			shape = visualization_msgs::Marker::ARROW;
			marker.color.r = 0.0f;
			break;
		case visualization_msgs::Marker::ARROW:
			shape = visualization_msgs::Marker::CYLINDER;
			marker.color.r = 1.0f;
			break;
		case visualization_msgs::Marker::CYLINDER:
			shape = visualization_msgs::Marker::CUBE;
			marker.color.r = 0.0f;
			break;
		}

		r.sleep();

	}
}
