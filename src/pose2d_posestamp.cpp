#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf/transform_broadcaster.h>
#include <string>

float x, y, th;

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    // laser macther position
    x  = msg->x;
	y  = msg->y;
	th = msg->theta;
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "pose2d_posestamp");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose2D_pose", 50);

    // Topic subscribers
    ros::Subscriber subPose = n.subscribe("/pose2D", 1000, &poseCallback);
	
	ros::Time current_time;
    current_time = ros::Time::now();

    while(n.ok())
    {
		current_time = ros::Time::now(); 
		geometry_msgs::PoseWithCovarianceStamped pose;
		pose.header.frame_id = "odom";
		pose.header.stamp = ros::Time::now();
		
		// set x,y coord
		pose.pose.pose.position.x = x;
		pose.pose.pose.position.y = y;
		pose.pose.pose.position.z = 0.0;
		
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
		pose.pose.pose.orientation = pose_quat;
		
		pose.pose.covariance[6*0+0] = 0.5 * 0.5;
		pose.pose.covariance[6*1+1] = 0.5 * 0.5;
		pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
		
		// publish
		pose_pub.publish(pose);
		ros::spinOnce();
    }
    return 0;
}
