#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <tf/tf.h>

using namespace std;

// typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
// typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;

class path_visualiser{
	public:
		path_visualiser();
	private:
		ros::Subscriber odom_sub, odom_combined_sub, imu_sub; 
		ros::Publisher marker_pub, imu_pub;
		visualization_msgs::Marker odom_line, combined_line;
		void odom_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& combined_data);
		void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_data);
		void imu_cb(const sensor_msgs::Imu::ConstPtr& imu);
};

path_visualiser::path_visualiser(){
	ros::Time::init();
	ros::NodeHandle n;

	odom_line.header.frame_id = "mymap";
    odom_line.header.stamp = ros::Time::now();
    odom_line.ns = "line";
    odom_line.id = 0;
    odom_line.action = visualization_msgs::Marker::ADD;
    odom_line.type = visualization_msgs::Marker::LINE_STRIP;
    odom_line.pose.orientation.w = 1.0;
    odom_line.scale.x = 0.1;
    odom_line.color.r = 1.0;
    odom_line.color.a = 1.0;

    combined_line.header.frame_id = "mymap";
    combined_line.header.stamp = ros::Time::now();
    combined_line.ns = "line";
    combined_line.id = 1;
    combined_line.action = visualization_msgs::Marker::ADD;
    combined_line.type = visualization_msgs::Marker::LINE_STRIP;
    combined_line.pose.orientation.w = 1.0;
    combined_line.scale.x = 0.1;
    combined_line.color.g = 1.0;
    combined_line.color.a = 1.0;

	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker", 10);
	imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 10);
	odom_sub = n.subscribe("/zed/odom", 10, &path_visualiser::odom_cb, this);
	imu_sub = n.subscribe("/imu/data", 10, &path_visualiser::imu_cb, this);
	odom_combined_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 10, &path_visualiser::odom_combined_cb, this);
}

void path_visualiser::odom_cb(const nav_msgs::Odometry::ConstPtr& odom_data){
	geometry_msgs::Point data_point;
	data_point = odom_data->pose.pose.position;
	odom_line.points.push_back(data_point);
    marker_pub.publish(odom_line);
    
}
void path_visualiser::imu_cb(const sensor_msgs::Imu::ConstPtr& imu){
	// transform imu frame to odometry frame

    tf::Quaternion transform_orientation, origin_orientation;
    quaternionMsgToTF(imu->orientation, origin_orientation);
    tf::Matrix3x3 matrixI2C, matrixC2O, origin_matrix, transform_matrix;
    tf::Vector3 origin_angular_velocity, origin_linear_acceleration, transform_angular_velocity, transform_linear_acceleration;

    origin_angular_velocity = tf::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    origin_linear_acceleration = tf::Vector3(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    matrixI2C = tf::Matrix3x3(-0.02252259, -0.99974486, -0.0017194,
                              -0.06487645, 0.00317777, -0.99788824,
                              0.9976391, -0.02236348, -0.06493147);
    matrixC2O = tf::Matrix3x3(0, 0, 1,
                              -1, 0, 0,
                              0, -1, 0);
    origin_matrix = tf::Matrix3x3(origin_orientation);
    //transform_matrix = origin_matrix * matrixI2C * matrixC2O;
    transform_matrix = matrixC2O * matrixI2C * origin_matrix;
    transform_matrix.getRotation(transform_orientation);
    sensor_msgs::Imu imu_transform_i2z = *imu;
    quaternionTFToMsg(transform_orientation, imu_transform_i2z.orientation);
    transform_angular_velocity = matrixC2O * matrixI2C * origin_angular_velocity;
    transform_linear_acceleration = matrixC2O * matrixI2C * origin_linear_acceleration;
    vector3TFToMsg(transform_angular_velocity, imu_transform_i2z.angular_velocity);
    vector3TFToMsg(transform_linear_acceleration, imu_transform_i2z.linear_acceleration);
    imu_pub.publish(imu_transform_i2z);
}
void path_visualiser::odom_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& combined_data){
	geometry_msgs::Point data_point;
	data_point = combined_data->pose.pose.position;
	combined_line.points.push_back(data_point);
	marker_pub.publish(combined_line);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hw4_node");
    path_visualiser path_visualiser;
    ros::spin();
    return 0;
}