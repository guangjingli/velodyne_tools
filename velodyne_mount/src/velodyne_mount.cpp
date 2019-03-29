//
// Created by lgj on 19-3-29.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

float front_behind = 10;
float left_right = 10;
float grid = 2;

ros::Publisher pointsPub;

void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &in_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in_points, *points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_selected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> point_front;
    pcl::PointCloud<pcl::PointXYZ> point_behind;
    pcl::PointCloud<pcl::PointXYZ> point_left;
    pcl::PointCloud<pcl::PointXYZ> point_right;
    pcl::PointXYZ center_front, center_behind, center_left, center_right;

    for(auto pt : points->points){
        //前
        if(pt.x>front_behind-grid/2.0 && pt.x<front_behind+grid/2.0 && pt.y < grid/2.0 && pt.y>-grid/2.0){
            points_selected->points.push_back(pt);
            point_front.points.push_back(pt);
            center_front.x += pt.x;
            center_front.y += pt.y;
            center_front.z += pt.z;
        }
        //后
        if(pt.x>-(front_behind+grid/2.0) && pt.x<-(front_behind-grid/2.0) && pt.y < grid/2.0 && pt.y>-grid/2.0){
            points_selected->points.push_back(pt);
            point_behind.points.push_back(pt);
            center_behind.x += pt.x;
            center_behind.y += pt.y;
            center_behind.z += pt.z;
        }
        //左
        if(pt.x>-grid/2.0 && pt.x<grid/2.0 && pt.y>left_right-grid/2.0 && pt.y<left_right+grid/2.0){
            points_selected->points.push_back(pt);
            point_left.points.push_back(pt);
            center_left.x += pt.x;
            center_left.y += pt.y;
            center_left.z += pt.z;
        }
        //右
        if(pt.x>-grid/2.0 && pt.x<grid/2.0 && pt.y>-(left_right+grid/2.0) && pt.y<-(left_right-grid/2.0)){
            points_selected->points.push_back(pt);
            point_right.points.push_back(pt);
            center_right.x += pt.x;
            center_right.y += pt.y;
            center_right.z += pt.z;
        }
    }

    center_front.x /= point_front.points.size();
    center_front.y /= point_front.points.size();
    center_front.z /= point_front.points.size();

    center_behind.x /= point_behind.points.size();
    center_behind.y /= point_behind.points.size();
    center_behind.z /= point_behind.points.size();

    center_left.x /= point_left.points.size();
    center_left.y /= point_left.points.size();
    center_left.z /= point_left.points.size();

    center_right.x /= point_right.points.size();
    center_right.y /= point_right.points.size();
    center_right.z /= point_right.points.size();

    float pitch = atan2(center_front.z - center_behind.z, center_front.x - center_behind.x) * 180 / M_PI;
    float roll = atan2(center_left.z - center_right.z, center_left.y - center_right.y) * 180 / M_PI;
    std::cout << std::endl;

    std::cout << "front_z: " << center_front.z << "\tbehind_z: " << center_behind.z << "\tpitch: " << pitch << std::endl;
    std::cout << "left_z: " << center_left.z << "\tright_z: " << center_right.z << "\troll: " << roll << std::endl;

    sensor_msgs::PointCloud2 points_pub_msg;
    pcl::toROSMsg(*points_selected, points_pub_msg);
    points_pub_msg.header.frame_id = "velodyne";
    points_pub_msg.header.stamp = ros::Time::now();
    pointsPub.publish(points_pub_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "velodyne_mount_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<float>("front_behind", front_behind, 10);
    pnh.param<float>("left_right", left_right, 10);
    pnh.param<float>("grid", grid, 2);

    pointsPub = nh.advertise<sensor_msgs::PointCloud2>("/points_selected", 2);
    ros::Subscriber pointSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, velodyneCallback);

    ros::spin();
    return 1;
}


