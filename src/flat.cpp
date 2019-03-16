#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include "pcl_ros/point_cloud.h"

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#define pi 3.14159265359 

//using namespace grid_map;
typedef pcl::PointXYZ PointT;
ros::Publisher point_pub;
ros::Publisher normal_pub;
ros::Publisher pub_transformed_cloud;
int count = 0;

void show_normal(pcl::PointXYZ pot1, pcl::PointXYZ pot2, pcl::PointXYZ pot3, std::vector<double> pointvector,visualization_msgs::MarkerArray *markers) 
{
    visualization_msgs::Marker mk;
    mk.header.frame_id = "/aidinvi_body";
    mk.header.stamp = ros::Time::now();

    mk.type = visualization_msgs::Marker::ARROW; //arrow
    mk.action = visualization_msgs::Marker::ADD;
    int size = pointvector.size();
   // ROS_INFO("%d", size);
    mk.points.resize(2);

    mk.points[0].x = (pot1.x + pot2.x + pot3.x)/3;
    mk.points[0].y = (pot1.y + pot2.y + pot3.y)/3;
    mk.points[0].z = (pot1.z + pot2.z + pot3.z)/3;

    //ROS_INFO("pointfine");
    double vector_x = pointvector[0];
    double vector_y = pointvector[1];
    double vector_z = pointvector[2];
    double absolute = sqrt(vector_x*vector_x + vector_y*vector_y + vector_z*vector_z);

    mk.points[1].x = 0.1*pointvector[0]/absolute + mk.points[0].x;
    mk.points[1].y = 0.1*pointvector[1]/absolute + mk.points[0].y;
    mk.points[1].z = 0.1*pointvector[2]/absolute + mk.points[0].z;

    mk.scale.x = 0.005;
    mk.scale.y = 0.010;
    mk.scale.z = 0.0;
    
    mk.color.r = 255;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.color.a = 1;

    mk.lifetime = ros::Duration();
    mk.ns = "normalvector";
    mk.id = count;
    count++;
    if(count >= 40*30) count = 0;
    markers->markers.push_back(mk);
}

std::vector<double> normalcalculate(pcl::PointXYZ pot1, pcl::PointXYZ pot2, pcl::PointXYZ pot3)
{
   // ROS_INFO("vectorbefore");
    std::vector<double> pointvector;
    pointvector.resize(3);
    int size = pointvector.size();
    //ROS_INFO("%d", size);
    //ROS_INFO("vectorafter");
    pointvector[0] = (pot2.y - pot1.y)*(pot3.z - pot1.z) - (pot3.y - pot1.y)*(pot2.z - pot1.z);
    pointvector[1] = -(pot2.x - pot1.x)*(pot3.z - pot1.z) + (pot3.x - pot1.x)*(pot2.z - pot1.z);
    pointvector[2] = (pot2.x - pot1.x)*(pot3.y - pot1.y) - (pot3.x - pot1.x)*(pot2.y - pot1.y);
    //ROS_INFO("showbefore");
    return pointvector;
    //show_normal(pot1, pot2, pot3, pointvector);
    //ROS_INFO("showafter");
}

void flatcalculate(pcl::PointCloud<PointT> &rawpointcloud)
{
    pcl::PointXYZ point1, point2, point3, point4;
    
    visualization_msgs::MarkerArray markers;
    //markers.markers.resize(4800);
    std::vector<double> pointvector2;
    for(int i=0; i<30*40-40; i = i+40) {
        for(int j=0; j<40-1; j++) {
            point1 = rawpointcloud.points[i+j];
            point2 = rawpointcloud.points[i+j+1];
            point3 = rawpointcloud.points[i+j+40];
            point4 = rawpointcloud.points[i+j+1+40];
            //ROS_INFO("normalbefore"); 
            //if(i % 40*5 == 0 && j % 10 == 0) {
                pointvector2 = normalcalculate(point1, point2, point3);
                show_normal(point1, point2, point3, pointvector2, &markers);
           // }
            /*if(i % 5 == 0 && j % 10 == 0) {
                pointvector2 = normalcalculate(point1, point2, point4);           
                show_normal(point1, point2, point4, pointvector2, &markers);
            }*/
            //ROS_INFO("normalafter");
        }
    }
    normal_pub.publish(markers);
}

void msgCallbackpoint(const sensor_msgs::PointCloud2ConstPtr &pointcloudraw) 
{
    pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (*pointcloudraw, *transformed_cloud);
    pcl::PointCloud<PointT>::Ptr point_cloud (new pcl::PointCloud<PointT>);
    //point_pub.publish(pointcloudraw);
    pcl::PointXYZ pt2;
    int num = 40*30;
    //ROS_INFO("flatbefore");
    //ROS_INFO("flatafter");
    for(int i=0; i<num; i++) {
        pt2 = transformed_cloud->points[i];
        point_cloud->points.push_back(pt2);
    }
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    //transform_1.translation() << 0, neckJoint2CameraCentery, neckJoint2CameraCenterz;
    transform_1.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    transform_1.rotate (Eigen::AngleAxisf (-(90+80)*pi/180, Eigen::Vector3f::UnitX()));
    //transform_1.rotate (Eigen::AngleAxisf (camera_link_roll, Eigen::Vector3f::UnitX()));

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    //transform_2.translation() << body2neckJointx, body2neckJointy, body2neckJointz;//for gazebo
    transform_2.translation() << 0, 0.485, -0.038523;

    pcl::transformPointCloud (*point_cloud, *point_cloud, transform_1); //for real
    pcl::transformPointCloud (*point_cloud, *point_cloud, transform_2);

    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_1); //for real
    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_2);

    int transformed_cloud_pt_size = transformed_cloud->size();
    //foothold_pt->header.frame_id = "camera_depth_optical_frame";
    //transformed_cloud->header.frame_id = "world";
    transformed_cloud->header.frame_id = "aidinvi_body";
    transformed_cloud->width = transformed_cloud_pt_size;  //width should be same with push_back msg size
    transformed_cloud->height = 1;
    pcl_conversions::toPCL(ros::Time::now(), transformed_cloud->header.stamp);

    //pub_transformed_cloud.publish(transformed_cloud);

    flatcalculate(*transformed_cloud);
    
    point_cloud->header.frame_id = "aidinvi_body";
    point_cloud->width = num;
    point_cloud->height = 1;
    pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);

    point_pub.publish(point_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "flat");
    ros::NodeHandle nh;
    ros::Subscriber point_sub = nh.subscribe("/aidinvi_camera/depth/points", 1, msgCallbackpoint);
    point_pub = nh.advertise<pcl::PointCloud<PointT>>("check", 1);
    //pub_transformed_cloud = nh.advertise<pcl::PointCloud<PointT>>("check2", 1);
    normal_pub = nh.advertise<visualization_msgs::MarkerArray>("normal_vector", 1);

    ros::spin();
}