#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <math.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;

class icp_localization{
	public:
		icp_localization();
	private:
		int count;
		Eigen::Matrix4f initial_guess;
		ros::Subscriber point_cloud_sb; 
		ros::Publisher map_pub;
		ros::Publisher scan_pub, original_map_pub;
		PointCloud<PointXYZRGB>::Ptr registered_cloud, original_map_cloud;
		PointCloud<PointXYZRGB>::Ptr map_cloud;
		void point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_data);
		void point_cloud_preprocesisng_map(PointCloud<PointXYZRGB>::Ptr noised_cloud_data);
		void point_cloud_preprocesisng_scan(PointCloud<PointXYZRGB>::Ptr noised_cloud_data); 
		void load_map(void);
		//Eigen::Matrix4f initial_guess(PointCloud<PointXYZRGB>::Ptr cloud_src, PointCloud<PointXYZRGB>::Ptr cloud_target);
};
icp_localization::icp_localization(){
	ros::Time::init();
	ros::NodeHandle n;
	count = 1;
	initial_guess = Eigen::Matrix4f::Identity();
	initial_guess <<    -0.879, -0.573, 0, -2.2,
					    0.573,  -0.879, 0,  7.5,
						0    ,  0,      1, -2.7,
						0    ,  0,      0,    1;
	//downsampled_cloud.reset(new PointCloud<PointXYZRGB>());
	//denoised_cloud.reset(new PointCloud<PointXYZRGB>());
	map_cloud.reset(new PointCloud<PointXYZRGB>());
	registered_cloud.reset(new PointCloud<PointXYZRGB>());
	original_map_cloud.reset(new PointCloud<PointXYZRGB>());

	original_map_pub = n.advertise<sensor_msgs::PointCloud2>("original_map_cloud", 10);
	//map_pub = n.advertise<sensor_msgs::PointCloud2>("preprocessed_map_cloud", 1);
	scan_pub = n.advertise<sensor_msgs::PointCloud2>("scan_cloud", 10);

	point_cloud_sb = n.subscribe<sensor_msgs::PointCloud2>("points_raw", 10, &icp_localization::point_cloud_cb, this);
}
void icp_localization::load_map(){
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ros::package::getPath("hw5_0410815")+"/map.pcd", *map_cloud) == -1){
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return;
    }
  	//cout << "Loaded " << map_cloud->width * map_cloud->height << " data points from map.pcd with the following fields: " << std::endl;
  	//map_cloud->header.frame_id = "map";
  	//map_cloud->header.stamp.sec = ros::Time::now().to_sec();
}
void icp_localization::point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_data){
	PointCloud<PointXYZRGB>::Ptr original_cloud(new PointCloud<PointXYZRGB>);
	pcl::fromROSMsg (*point_cloud_data, *original_cloud);
	load_map();
	copyPointCloud(*map_cloud, *original_map_cloud);

	//Preprocessing
	// point_cloud_preprocesisng_scan(original_cloud);
	// point_cloud_preprocesisng_map(map_cloud);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
  	pass.setInputCloud (original_cloud);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (0, 1);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*original_cloud);

	pcl::VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud(map_cloud);
	sor.setLeafSize(0.3f, 0.3f, 0.3f);
	sor.filter(*map_cloud);
	//copyPointCloud(*noised_cloud_data, *downsampled_cloud);

	// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
	// sor2.setInputCloud(map_cloud);
	// sor2.setMeanK(100);
	// sor2.setStddevMulThresh(0.1);
	// sor2.filter(*map_cloud);


	//Initial guess
	// Eigen::Matrix4f scan2map_tf = initial_guess(original_cloud, map_cloud);
	// pcl::transformPointCloud (*original_cloud, *original_cloud, scan2map_tf);
	
	//pcl::transformPointCloud(*original_cloud, *original_cloud, initial_guess);




	//Starting ICP
	ROS_INFO("-----ICP Start-----\n");
	cout << "Loaded " << map_cloud->width * map_cloud->height << " data points to do ICP." << std::endl;

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	if (count >= 5){
		icp.setMaximumIterations ( 3000);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.01);
		
	}
	count ++;
	icp.setMaxCorrespondenceDistance(10);
	icp.setInputSource(original_cloud);
	icp.setInputTarget(map_cloud);
	icp.align(*registered_cloud, initial_guess);
	if ( icp.hasConverged() ){
  		cout << "Icp score: " << icp.getFitnessScore() << endl;
  	}
  	ROS_INFO("-----ICP Finish-----\n");
  	//registered_cloud->header.frame_id = "scan";
  	Eigen::Matrix4f s2m_transform = icp.getFinalTransformation();

  	initial_guess = s2m_transform ;
  	//s2m_transform = scan2map_tf * s2m_transform;

  	static tf::TransformBroadcaster br;
  	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(s2m_transform(0,3), s2m_transform(1,3), s2m_transform(2,3)));
  	tf::Quaternion q;
	tf::Matrix3x3 tf_rot = tf::Matrix3x3(s2m_transform(0,0), s2m_transform(0,1), s2m_transform(0,2),
                                         s2m_transform(1,0), s2m_transform(1,1), s2m_transform(1,2),
                                         s2m_transform(2,0), s2m_transform(2,1), s2m_transform(2,2));
	tf_rot.getRotation(q);
	//q.normalize();
	transform.setRotation(q);
	

	//copyPointCloud(*original_cloud, *registered_cloud);

	sensor_msgs::PointCloud2 map_msg, scan_msg;
	int i =0;
	for(i =0; i< original_map_cloud->size(); i++){
		original_map_cloud->points[i].g = 255;
	}
	for(i = 0; i < original_cloud->size(); i++){
		original_cloud->points[i].r = 255;
	}
	pcl::toROSMsg(*original_map_cloud, map_msg);
	pcl::toROSMsg(*original_cloud, scan_msg);
	map_msg.header.frame_id = "map";
	map_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "scan";
	scan_msg.header.stamp = ros::Time::now();
	//original_map_cloud = pcl::PointCloud<PointXYZRGB>::PointXYZRGB(0,255,0);
	//original_map_cloud->header.stamp = registered_cloud->header.stamp;
	original_map_pub.publish(map_msg);
  	//map_pub.publish(map_cloud);	
  	scan_pub.publish(scan_msg);
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scan"));
}
void icp_localization::point_cloud_preprocesisng_map(PointCloud<PointXYZRGB>::Ptr noised_cloud_data){

	// vector<int> indices;
 //    pcl::removeNaNFromPointCloud(*noised_cloud_data, *noised_cloud_data, indices);

	

	// pcl::PassThrough<pcl::PointXYZRGB> pass;
 //  	pass.setInputCloud (noised_cloud_data);
 //  	pass.setFilterFieldName ("z");
 //  	pass.setFilterLimits (-1.0, 4.0);
 //  	//pass.setFilterLimitsNegative (true);
 //  	pass.filter (*noised_cloud_data);

 //  	pass.setInputCloud (noised_cloud_data);
 //  	pass.setFilterFieldName ("x");
 //  	pass.setFilterLimits (-100.0, 100.0);
 //  	pass.filter (*noised_cloud_data);

  	// pass.setInputCloud (noised_cloud_data);
  	// pass.setFilterFieldName ("y");
  	// pass.setFilterLimits (-50.0, 50.0);
  	// pass.filter (*noised_cloud_data);

	// vector<int> indices2;
 //  	pcl::removeNaNFromPointCloud(*noised_cloud_data, *noised_cloud_data, indices2);
	//copyPointCloud(*noised_cloud_data, *denoised_cloud);

	ROS_INFO("Filtered map point cloud size: %d\n", noised_cloud_data->size());
	return;
}
void icp_localization::point_cloud_preprocesisng_scan(PointCloud<PointXYZRGB>::Ptr noised_cloud_data){

	// vector<int> indices;
 //    pcl::removeNaNFromPointCloud(*noised_cloud_data, *noised_cloud_data, indices);

	// pcl::VoxelGrid<PointXYZRGB> sor;
	// sor.setInputCloud(noised_cloud_data);
	// sor.setLeafSize(0.15f, 0.15f, 0.15f);
	// sor.filter(*noised_cloud_data);
	// //copyPointCloud(*noised_cloud_data, *downsampled_cloud);

	// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
	// if (noised_cloud_data->points.size()>100){
	// 	sor2.setInputCloud(noised_cloud_data);
	// 	sor2.setMeanK(100);
	// 	sor2.setStddevMulThresh(0.1);
	// 	sor2.filter(*noised_cloud_data);
	// }

	pcl::PassThrough<pcl::PointXYZRGB> pass;
  	pass.setInputCloud (noised_cloud_data);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (-1.5, 4.0);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*noised_cloud_data);

	// vector<int> indices2;
 //  	pcl::removeNaNFromPointCloud(*noised_cloud_data, *noised_cloud_data, indices2);
	//copyPointCloud(*noised_cloud_data, *denoised_cloud);
	ROS_INFO("Filtered scan point cloud size: %d\n", noised_cloud_data->size());
	return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hw5_node");
    icp_localization icp_localization;
    ros::spin();
    return 0;
}