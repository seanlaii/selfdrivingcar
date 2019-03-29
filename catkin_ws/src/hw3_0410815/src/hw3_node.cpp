#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <math.h>
using namespace std;

class path_visualiser{
    public:
        path_visualiser();
    private:
        ros::Publisher marker_pub;
        ros::Subscriber imu_sub;
        Eigen::Vector3d s_before;
        Eigen::Vector3d v_before;
        Eigen::Vector3d g;
        Eigen::Matrix3d C_before;
        double t_before;
        int count;
        visualization_msgs::Marker line;
        void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_data);       
};
path_visualiser::path_visualiser(){
    ros::Time::init();
    ros::NodeHandle n;
    s_before << 0.0,0.0,0.0;
    v_before << 0.0,0.0,0.0;
    cout << "v_before" << v_before << "\n";
    g << 0.0,0.0,0.0;
    C_before = Eigen::MatrixXd::Identity(3, 3);
    t_before = 0;
    count = 0;
    
    line.header.frame_id = "mymap";
    line.header.stamp = ros::Time::now();
    line.ns = "line";
    line.id = 0;
    line.action = visualization_msgs::Marker::ADD;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.1;
    line.color.b = 1.0;
    line.color.a = 1.0;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/marker", 10);
    imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 1, &path_visualiser::imu_cb, this);
}
void path_visualiser::imu_cb(const sensor_msgs::Imu::ConstPtr& imu_data){
    //Calculate the path of IMU
    geometry_msgs::Point p;
    std_msgs::Header header = imu_data->header;
    double t_now = header.stamp.toSec();
    double dt = 0;
    //cout << t_now;
    if (t_before != 0){
        dt = t_now - t_before;
    
        //cout << dt;
        Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::Matrix3d C_now, B;
        geometry_msgs::Vector3 w = imu_data->angular_velocity;
        geometry_msgs::Vector3 a = imu_data->linear_acceleration;
        
        
        if(count == 0){
            g[0] = a.x;
            g[1] = a.y;
            g[2] = a.z;
            count ++;
            ROS_INFO("Start Calculating");
        }
        double sigma;
        Eigen::Vector3d v_now, s_now, ag, ab;
        sigma = sqrt(w.x*w.x*dt*dt + w.y*w.y*dt*dt + w.z*w.z*dt*dt);
        ab << a.x, a.y, a.z;
        B << 0,     -1*w.z*dt,        w.y*dt,
            w.z*dt,        0,     -1*w.x*dt,
            -1*w.y*dt,     w.x*dt,        0;
        C_now = C_before*(I+(sin(sigma)/sigma)*B+((1-cos(sigma))/(sigma*sigma))*B*B);
        
        ag = C_now*ab;
        cout << "ag: " << ag << "\n";
        cout << "dt: " << dt << "\n";
        v_now = v_before + dt*(ag-g);
        cout << "v_before" << v_before << "\n";
        s_now = s_before + dt*v_now;
        cout << "s_now" << s_now << "\n";
        
        p.x = s_now[0];
        p.y = s_now[1];
        p.z = s_now[2];
        ROS_INFO("Point = %f, %f, %f", p.x, p.y, p.z);

        //Visualization by marker
        
        line.points.push_back(p);
        marker_pub.publish(line);
        
        v_before = v_now;
        s_before = s_now;
        C_before = C_now;
    }
    t_before = t_now;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hw3_node");
    path_visualiser path_visualiser;
    ros::spin();
    return 0;
}