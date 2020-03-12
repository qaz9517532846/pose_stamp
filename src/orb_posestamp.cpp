#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf/transform_broadcaster.h>
#include <string>

double tx, ty, tz;
double roll, pitch, yaw;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	
    // Camera position in map frame
    tx = msg->pose.position.x - 0.26;
    ty = msg->pose.position.y - 0.05;
    tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(roll, pitch, yaw);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "orb_posestamp");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam_pose", 50);

    // Topic subscribers
    ros::Subscriber subPose = n.subscribe("/orb_slam2_rgbd/pose", 10, &poseCallback);
	
	ros::Time current_time;
    current_time = ros::Time::now();

    while(n.ok())
    {
		current_time = ros::Time::now(); 
		geometry_msgs::PoseWithCovarianceStamped pose;
		pose.header.frame_id = "odom";
		pose.header.stamp = ros::Time::now();
		
		// set x,y coord
		pose.pose.pose.position.x = tx;
		pose.pose.pose.position.y = ty;
		pose.pose.pose.position.z = 0.0;
		
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(yaw);
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
