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
#include <sstream>
using namespace std;
using namespace pcl;

class map_construction{
	public:
		map_construction();
	private:
		Eigen::Matrix4f initial_guess;
		ros::Publisher map_pub;
		PointCloud<PointXYZ>::Ptr map_cloud, scan_cloud;
		PointCloud<PointXYZ> map, scan, final, filtered_cloud;
		void mapping(void);
};
map_construction::map_construction(){
	ros::Time::init();
	ros::NodeHandle n;
	map_cloud.reset(new PointCloud<PointXYZ>());
	scan_cloud.reset(new PointCloud<PointXYZ>());
	initial_guess = Eigen::Matrix4f::Identity();
	map_pub = n.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
	mapping();
}
void map_construction::mapping(void){
	PointCloud<PointXYZ>::Ptr registered_cloud;
	registered_cloud.reset(new PointCloud<PointXYZ>());
	int i, count;
	count = 0;
	for(i = 1; i <= 179; i++){
		//Load scan.pcd
		std::string out_string;
		std::stringstream ss;
		ss << i;
		out_string = ss.str();
		cout << "PCD file: "<< out_string << endl;
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("hw6_0410815")+"/pcd/"+ out_string +".pcd", scan) == -1){
	    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	    	return;
	    }
	    //cout << scan_cloud->size() << endl;
	    if(i == 1){
	    	//map_cloud = scan_cloud;
	    	map = scan;
	    	cout << "Map is constructed." << endl;
	    }
	    if(i != 1){

	    	pcl::PassThrough<pcl::PointXYZ> pass;
		  	pass.setInputCloud (scan.makeShared());
		  	pass.setFilterFieldName ("z");
		  	pass.setFilterLimits (0, 2);
		  	//pass.setFilterLimitsNegative (true);
		  	pass.filter (filtered_cloud);

		  	cout << "Scan after PassThrough: " << filtered_cloud.size() << endl;
	        pcl::VoxelGrid<PointXYZ> sor;
			sor.setInputCloud(filtered_cloud.makeShared());
			sor.setLeafSize(0.15f, 0.15f, 0.15f);
			sor.filter(filtered_cloud);

			cout << "Scan after VoxelGrid: " << filtered_cloud.size() << endl;
		    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setMaximumIterations ( 3000);
			icp.setTransformationEpsilon(1e-10);
			icp.setEuclideanFitnessEpsilon(0.01);
			icp.setMaxCorrespondenceDistance(10);
			icp.setInputSource(filtered_cloud.makeShared());
			icp.setInputTarget(map.makeShared());
			icp.align(final, initial_guess);
			if ( icp.hasConverged() ){
		  		cout << "Icp score: " << icp.getFitnessScore() << endl;
  			}
  			initial_guess = icp.getFinalTransformation();

  			//pcl::concatenateFields(*map_cloud, *registered_cloud, *map_cloud);
  			pcl::transformPointCloud (scan, scan, initial_guess);
  			//scan = * registered_cloud;
  			


	    }
	    if(i % 22 == 0 || i == 1){

	    	cout << "Before: " << map.size() << endl;
  			map += scan;
  			//*map_cloud = *map_cloud + *scan_cloud;
  			cout << "After: " << map.size() << endl;

	    	sensor_msgs::PointCloud2 map_msg;
	    	pcl::toROSMsg(map, map_msg);
	    	map_msg.header.frame_id = "map";
	    	map_msg.header.stamp = ros::Time::now();
	    	map_pub.publish(map_msg);
	    }
	    pcl::VoxelGrid<PointXYZ> sor1;
		sor1.setInputCloud(map.makeShared());
		sor1.setLeafSize(0.2f, 0.2f, 0.2f);
		sor1.filter(map);
	    
	}
	

	cout << "Size of map after voxel_grid: " << map.size() << endl;	
    //Save map.pcd
    pcl::io::savePCDFileASCII ("map.pcd", map);
    std::cerr << "Saved " << map.size () << " data points to map.pcd." << std::endl;
}
int main(int argc, char** argv){
	ros::init(argc, argv, "hw6_node");
	map_construction map_construction;
	ros::spin();
	return 0;
}
