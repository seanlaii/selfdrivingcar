#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
using namespace std;
using namespace pcl;

class cluster_classifier{
	public:
		cluster_classifier();
	private:
		visualization_msgs::MarkerArray texts;
		//cv_bridge::CvImagePtr img_ptr;
		PointCloud<pcl::PointXYZI>::Ptr cloud;
		//darknet_ros_msgs::BoundingBoxes::ConstPtr boxes;
		Eigen::Matrix4f matrix_L2C;
		cv::Mat intrinsic_matrix;
		cv::Mat distortion_coefficients;
		cv::Mat rVec;
		cv::Mat tVec;
		ros::Publisher marker_pub;
		ros::Subscriber box_sub, pc_sub;
		//void img_callback(const sensor_msgs::CompressedImage::ConstPtr& img_msg);
		void box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes);
		void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
};
cluster_classifier::cluster_classifier(){
	ros::Time::init();
	ros::NodeHandle n;
	intrinsic_matrix.create(3, 3, cv::DataType<double>::type);
	distortion_coefficients.create(5, 1, cv::DataType<double>::type);
	rVec.create(3, 1, cv::DataType<double>::type);
	tVec.create(3, 1, cv::DataType<double>::type);

	matrix_L2C << 0.845929, 0.533284, -0.003308, 0.092240,
				 0.045996, -0.079141, -0.995802, -0.357097,
				 -0.531307, 0.842226, -0.091477, -0.160559,
				 0, 0, 0, 1;

	intrinsic_matrix.at<double>(0,0) = 698.939;
	intrinsic_matrix.at<double>(1,0) = 0;
	intrinsic_matrix.at<double>(2,0) = 0;
	intrinsic_matrix.at<double>(0,1) = 0;
	intrinsic_matrix.at<double>(1,1) = 698.939;
	intrinsic_matrix.at<double>(2,1) = 0;
	intrinsic_matrix.at<double>(0,2) = 641.868;
	intrinsic_matrix.at<double>(1,2) = 385.788;
	intrinsic_matrix.at<double>(2,2) = 1;

	distortion_coefficients.at<double>(0) = -0.171466;
	distortion_coefficients.at<double>(1) = 0.0246144;
	distortion_coefficients.at<double>(2) = 0;
	distortion_coefficients.at<double>(3) = 0;
	distortion_coefficients.at<double>(4) = 0;

	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;
	//pc_pub = n.advertise<sensor_msgs::PointCloud2>("test_cloud", 10);
	//cluster_cloud.reset(new PointCloud<PointXYZI>());
	cloud.reset(new PointCloud<PointXYZI>());
	//boxes = new darknet_ros_msgs::BoundingBoxes();
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("texts_Marker", 10);
	//box_sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &cluster_classifier::box_callback, this);
	//img_sub = n.subscribe("/zed/left/image_raw_color/compressed", 1, &cluster_classifier::img_callback, this);
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("points_raw", 1, &cluster_classifier::pc_callback, this);
}
// void cluster_classifier::img_callback(const sensor_msgs::CompressedImage::ConstPtr& img_msg){
// 	img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
// }
// void cluster_classifier::box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg){
// 	boxes = box_msg;
// 	cout << "Loaded boxes" << endl;	
// }
void cluster_classifier::pc_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
	darknet_ros_msgs::BoundingBoxes::ConstPtr boxes = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",ros::Duration());
	int count = 0;
	fromROSMsg(*pc_msg, *cloud);
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.1f, 0.1f, 0.1f);
	vg.filter (*cloud_filtered);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.05);

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0) {
	std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Get the points associated with the planar surface
	extract.filter (*cloud_plane);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_f);
	*cloud_filtered = *cloud_f;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (2); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (1500);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
	std::cout << "Cluster size: " << cluster_indices.size() << std::endl;

	int j = 50;
	int k = 0;
	PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new PointCloud<pcl::PointXYZI>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      		cloud_filtered->points[*pit].intensity = j;
      		cluster_cloud->points.push_back (cloud_filtered->points[*pit]); 
    	}
    	PointCloud<pcl::PointXYZI>::Ptr center_point_cloud(new PointCloud<PointXYZI>);
    	Eigen::Vector4f centroid;
    	compute3DCentroid(*cluster_cloud, centroid);
    	PointXYZI center_point;
    	center_point.x = centroid[0];
    	center_point.y = centroid[1];
    	center_point.z = centroid[2];
    	center_point_cloud->points.push_back(center_point);
    	//cout << "Computing centroid accomplished." << endl;

    	transformPointCloud(*center_point_cloud, *center_point_cloud, matrix_L2C);
    	vector<cv::Point2f> imagePoints;
    	vector<cv::Point3f> lidarPoints;
    	lidarPoints.push_back(cv::Point3f(center_point_cloud->points[0].x, center_point_cloud->points[0].y, center_point_cloud->points[0].z));
    	projectPoints(lidarPoints, rVec, tVec, intrinsic_matrix, distortion_coefficients, imagePoints);
    	//cout << "Projection accomplished." << endl;

    	for(int i = 0 ; i < boxes->bounding_boxes.size(); i++){
    		//cout << "In the for loop." << endl;
    		if(imagePoints[0].x > boxes->bounding_boxes[i].xmin && imagePoints[0].x < boxes->bounding_boxes[i].xmax && imagePoints[0].y > boxes->bounding_boxes[i].ymin && imagePoints[0].y < boxes->bounding_boxes[i].ymax){
    			//if(imagePoints[0].y > boxes->bounding_boxes[i].ymin && imagePoints[0].y < boxes->bounding_boxes[i].ymax){
    			cout << "Meet the requirement." << endl;
    			visualization_msgs::Marker marker;
    			marker.header.frame_id = "velodyne";
    			marker.header.stamp = ros::Time::now();
    			marker.ns = "texts";
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose.orientation.w = 1.0;
    			marker.id = count;
    			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    			marker.scale.z = 3;
    			marker.color.a = 1.0;
    			marker.color.r = 0;
    			marker.color.g = 0;
    			marker.color.b = 0;
    			marker.lifetime = ros::Duration(1);
    			marker.pose.position.x = center_point.x;
    			marker.pose.position.y = center_point.y;
    			marker.pose.position.z = center_point.z + 2;
    			marker.text = boxes->bounding_boxes[i].Class;
    			texts.markers.push_back(marker);

    			count ++;
    			cout << "Count: " << count << endl;
    			break;
    		}
    		
    	}
    	lidarPoints.clear();
    	imagePoints.clear();
	}
	marker_pub.publish(texts);
	texts.markers.clear();
}
int main(int argc, char** argv){
	ros::init(argc, argv, "hw7_node");
    cluster_classifier cluster_classifier;
    ros::spin();
    return 0;
}