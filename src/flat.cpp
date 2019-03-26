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
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32.h>

#define pi 3.14159265359 

//using namespace grid_map;
typedef pcl::PointXYZ PointT;
/*ros::Publisher point_pub;
ros::Publisher normal_pub;
ros::Publisher pub_transformed_cloud;
ros::Publisher axis_pub;

ros::Subscriber point_sub;*/

geometry_msgs::TransformStamped aidinvi_body;
geometry_msgs::TransformStamped camera_link;

pcl::PointXYZ aidinvi_body_pos;
pcl::PointXYZ camera_link_pos;
std::vector<double> camera_link_vec_y;
std::vector<double> camera_link_vec_x;
int count = 0;
int transformed_cloud_size_before;

double flat_angle = 30.0;

class Flat
{
    private:
    ros::NodeHandle nh_;

    ros::Publisher point_pub;
    ros::Publisher normal_pub;
    ros::Publisher pub_transformed_cloud;
    ros::Publisher axis_pub;
    ros::Publisher flat_pub;

    ros::Subscriber point_sub;

    //std::string point_cloud_topic_ = "/aidinvi_camera/depth/points";
    std::string point_cloud_topic_ = "/ros2pcl/foothold_region";

    public:

    public:
    Flat(ros::NodeHandle nh)
    : nh_(nh)
    {
        

        point_sub = nh.subscribe(point_cloud_topic_, 1, &Flat::msgCallbackpoint, this);
        //point_sub = nh_.subscribe("/ros2pcl/foothold_region", 1, &Flat::msgCallbackpoint, this);
        point_pub = nh.advertise<pcl::PointCloud<PointT>>("check", 1);
        //pub_transformed_cloud = nh.advertise<pcl::PointCloud<PointT>>("check2", 1);
        normal_pub = nh_.advertise<visualization_msgs::MarkerArray>("normal_vector", 1);
        axis_pub = nh.advertise<visualization_msgs::MarkerArray>("axis_vector", 1);
        flat_pub = nh.advertise<pcl::PointCloud<PointT>>("flat_surface", 1);
    }
      ~Flat()
    {

    }

    void show_vector(pcl::PointXYZ pot1, pcl::PointXYZ pot2, pcl::PointXYZ pot3, std::vector<double> pointvector,visualization_msgs::MarkerArray *markers, int max_value) 
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
        markers->markers.push_back(mk);

        if(count >= max_value) {
            while(count < transformed_cloud_size_before) {
                mk.scale.x = 0;
                mk.scale.y = 0;
                mk.scale.z = 0;
                mk.ns = "normalvector";
                mk.id = count;
                count++;
                markers->markers.push_back(mk);
            }
            count = 0;

        }
    }

    std::vector<double> normalcalculate(pcl::PointXYZ pot1, pcl::PointXYZ pot2, pcl::PointXYZ pot3)
    {
    // ROS_INFO("vectorbefore");
        std::vector<double> pointvector;
        pointvector.resize(3);
        //int size = pointvector.size();
        //ROS_INFO("%d", size);
        //ROS_INFO("vectorafter");
        pointvector[0] = (pot2.y - pot1.y)*(pot3.z - pot1.z) - (pot3.y - pot1.y)*(pot2.z - pot1.z);
        pointvector[1] = -(pot2.x - pot1.x)*(pot3.z - pot1.z) + (pot3.x - pot1.x)*(pot2.z - pot1.z);
        pointvector[2] = (pot2.x - pot1.x)*(pot3.y - pot1.y) - (pot3.x - pot1.x)*(pot2.y - pot1.y);

        //ROS_INFO("showbefore");
        return pointvector;
        //show_vector(pot1, pot2, pot3, pointvector);
        //ROS_INFO("showafter");
    }

    pcl::PointCloud<PointT>::Ptr normal_determine(pcl::PointXYZ normal_point, std::vector<double> normal_vector, pcl::PointCloud<PointT>::Ptr flat_cloud2) {
        //unit vector
        double absolute = sqrt(normal_vector[0]*normal_vector[0] + normal_vector[1]*normal_vector[1] + normal_vector[2]*normal_vector[2]);
        std::vector<double> unitvector;
        unitvector.resize(3);
        unitvector[0] = normal_vector[0]/absolute;
        unitvector[1] = normal_vector[1]/absolute;
        unitvector[2] = normal_vector[2]/absolute;
        //determine normal
        std::vector<double> zaxis_vector;
        zaxis_vector.resize(3);
        zaxis_vector = {0, 0, -1};
        
        double normal_cos = unitvector[0]*zaxis_vector[0] + unitvector[1]*zaxis_vector[1] + unitvector[2]*zaxis_vector[2];
        if(normal_cos > cos(flat_angle)) { //flat
            flat_cloud2->points.push_back(normal_point);
        }
        return flat_cloud2;
    }

    int findnextrow(int current, pcl::PointCloud<PointT> findpointcloud, int add) {
        int next_current = current;
        pcl::PointXYZ findp1;
        pcl::PointXYZ findp2;
        std::vector<double> findrow_vec;
        findrow_vec.resize(3);
        double vec_value;
        while(ros::ok()) {
            findp1 = findpointcloud.points[next_current];
            next_current = next_current+add;
            findp2 = findpointcloud.points[next_current];
            findrow_vec[0] = findp2.x - findp1.x;
            findrow_vec[1] = findp2.y - findp1.y;
            findrow_vec[2] = findp2.z - findp1.z;

            vec_value = camera_link_vec_x[0] * findrow_vec[0] + camera_link_vec_x[1] * findrow_vec[1] + 
                        camera_link_vec_x[2] * findrow_vec[2];
            if(add > 0 && vec_value < 0) return next_current;
            if(add < 0 && vec_value > 0) return next_current;
        }
    }

    int findclosepoint(int currentp, pcl::PointCloud<PointT> findclosepointcloud, int min_point) {
        pcl::PointXYZ closep1 = findclosepointcloud.points[currentp];
        pcl::PointXYZ closep2;
        double min;
        double min_before = 99;
        int min_point_p;
        int next_row_point2 = findnextrow(min_point, findclosepointcloud, 1);
        for(int k = min_point; k <next_row_point2; k++) {
            closep2 = findclosepointcloud.points[k];
            min = sqrt((closep1.x - closep2.x)*(closep1.x - closep2.x) 
                    + (closep1.y - closep2.y)*(closep1.y - closep2.y));
            if(min < min_before) {
                min_before = min;
                min_point_p = k;
            }
        }
        return min_point_p;
    }
    int row_count = 0;
    void flatcalculate(pcl::PointCloud<PointT> &rawpointcloud)
    {
        pcl::PointXYZ point1, point2, point3, point4;
        pcl::PointCloud<PointT>::Ptr flat_cloud (new pcl::PointCloud<PointT>);
        
        visualization_msgs::MarkerArray markers;
        //markers.markers.resize(4800);
        std::vector<double> pointvector2;
        int raw_size = rawpointcloud.size();
        //ROS_INFO("raw_size : %d", raw_size);
        int next_row_point;
        int last_row_point = findnextrow(raw_size-1, rawpointcloud, -1);
        //ROS_INFO("last_row_point : %d", last_row_point);
        row_count = 0;
        for(int i=0; i<last_row_point; i) { //raw_size need to change
            int next_row_point = findnextrow(i, rawpointcloud, 1);
            row_count++;
        // ROS_INFO("next_row_point : %d", next_row_point);
            for(int j=i; j<next_row_point-1; j++) {
                point1 = rawpointcloud.points[j];
                point2 = rawpointcloud.points[j+1];
                int close_point = findclosepoint(j, rawpointcloud, next_row_point);
            // ROS_INFO("current : %d, close_point : %d", j, close_point);
                point3 = rawpointcloud.points[close_point]; 
                //point4 = rawpointcloud.points[j+1+40];
                //ROS_INFO("normalbefore"); 
                //if(i % 40*5 == 0 && j % 10 == 0) {
                    pointvector2 = normalcalculate(point1, point2, point3);
                    flat_cloud = normal_determine(point1, pointvector2, flat_cloud);
                    show_vector(point1, point2, point3, pointvector2, &markers, last_row_point - row_count);
            // }
                //if(i % 5 == 0 && j % 10 == 0) {
                //  pointvector2 = normalcalculate(point1, point2, point4);           
                    //show_vector(point1, point2, point4, pointvector2, &markers, raw_size);
                //}
                //ROS_INFO("normalafter");
            }
            i = next_row_point;
        }
        flat_cloud->header.frame_id = "aidinvi_body";
        flat_cloud->width = flat_cloud->size();
        flat_cloud->height = 1;
        pcl_conversions::toPCL(ros::Time::now(), flat_cloud->header.stamp);

        flat_pub.publish(flat_cloud);

        //point_pub.publish(point_cloud);
        /*std::vector<double> testvector;
        testvector.resize(3);
        pcl::PointXYZ test1, test2;
        test1 = rawpointcloud.points[last_row_point+1];
        test2 = rawpointcloud.points[last_row_point];
        testvector[0] = test2.x - test1.x;
        testvector[1] = test2.y - test1.y;
        testvector[2] = test2.z - test1.z;
        show_vector(test1, test1, test1, testvector, &markers, 1);*/
        normal_pub.publish(markers);
    }

    void msgCallbackpoint(const sensor_msgs::PointCloud2ConstPtr &pointcloudraw) 
    {
    
        pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>);
        pcl::fromROSMsg (*pointcloudraw, *transformed_cloud);
        pcl::PointCloud<PointT>::Ptr point_cloud (new pcl::PointCloud<PointT>);
        //point_pub.publish(pointcloudraw);
        pcl::PointXYZ pt2;
        
        camera_link_vec_y.resize(3);
        camera_link_vec_x.resize(3);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        visualization_msgs::MarkerArray markers2;

        while(ros::ok()) {
            try{
                aidinvi_body = tfBuffer.lookupTransform("world", "aidinvi_body", ros::Time(0));
                camera_link = tfBuffer.lookupTransform("aidinvi_body", "camera_link", ros::Time(0));
                
                aidinvi_body_pos.x = aidinvi_body.transform.translation.x;
                aidinvi_body_pos.y = aidinvi_body.transform.translation.y;
                aidinvi_body_pos.z = aidinvi_body.transform.translation.z;

                camera_link_pos.x = camera_link.transform.translation.x;
                camera_link_pos.y = camera_link.transform.translation.y;
                camera_link_pos.z = camera_link.transform.translation.z;

                camera_link_vec_y[0] = camera_link_pos.x - aidinvi_body_pos.x;
                camera_link_vec_y[1] = camera_link_pos.y - aidinvi_body_pos.y;
                camera_link_vec_y[2] = 0;

                camera_link_vec_x[0] = camera_link_vec_y[1];
                camera_link_vec_x[1] = -camera_link_vec_y[0];
                camera_link_vec_x[2] = 0;

                //show_vector(camera_link_pos, camera_link_pos, camera_link_pos, camera_link_vec_y, &markers2, 2);
                //show_vector(camera_link_pos, camera_link_pos, camera_link_pos, camera_link_vec_x, &markers2, 2);
               // axis_pub.publish(markers2);

                break;
            }
            catch (tf2::TransformException &ex) {
            }
        }
        //int num = 40*30;
        int num = 10;
        //ROS_INFO("flatbefore");
        //ROS_INFO("flatafter");
        for(int i=0; i<num; i++) {
            pt2 = transformed_cloud->points[i];
            point_cloud->points.push_back(pt2);
        }
        
        if(point_cloud_topic_ == "/aidinvi_camera/depth/points") {
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
        }        
        
        int transformed_cloud_size = transformed_cloud->size();
        //foothold_pt->header.frame_id = "camera_depth_optical_frame";
        //transformed_cloud->header.frame_id = "world"; 
        transformed_cloud->header.frame_id = "aidinvi_body";
        transformed_cloud->width = transformed_cloud_size;  //width should be same with push_back msg size
        transformed_cloud->height = 1;
        pcl_conversions::toPCL(ros::Time::now(), transformed_cloud->header.stamp);

        //pub_transformed_cloud.publish(transformed_cloud);

        flatcalculate(*transformed_cloud);

        transformed_cloud_size_before = transformed_cloud_size;

        point_cloud->header.frame_id = "aidinvi_body";
        point_cloud->width = num;
        point_cloud->height = 1;
        pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);

        //point_pub.publish(point_cloud);

    }
};

void msgCallbackflat_angle(const std_msgs::Float32::ConstPtr& msg) 
{
    flat_angle = (double)msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "flat");
    ros::NodeHandle nh;
    ros::Subscriber flat_angle_sub = nh.subscribe("flat_angle", 1, msgCallbackflat_angle);
    Flat flat(nh);

    /*//ros::Subscriber point_sub = nh.subscribe("/aidinvi_camera/depth/points", 1, msgCallbackpoint);
    point_sub = nh.subscribe("/ros2pcl/foothold_region", 1, msgCallbackpoint);
    //point_pub = nh.advertise<pcl::PointCloud<PointT>>("check", 1);
    //pub_transformed_cloud = nh.advertise<pcl::PointCloud<PointT>>("check2", 1);
    normal_pub = nh.advertise<visualization_msgs::MarkerArray>("normal_vector", 1);
    //axis_pub = nh.advertise<visualization_msgs::MarkerArray>("axix_vector", 1);*/
    ros::spin();
}