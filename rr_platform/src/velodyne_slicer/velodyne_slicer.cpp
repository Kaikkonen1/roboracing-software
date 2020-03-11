//
// Created by nico on 3/2/20.
//

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void pointcloudCallback(const sensor_msgs::PointCloud2 cloud_msg) { //could be a const ptr
    float threshold = 0.05;
    float height = 1; //height in meters off of ground?

    //container for original & filtered data
    pcl::PCLPointCloud2 cloud_pcl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZ>);

    //convert to plc data type
    pcl_conversions::toPCL(cloud_msg, cloud_pcl);
    pcl::fromPCLPointCloud2(cloud_pcl, *cloud_pc);

    //Perform the filtering
    for (auto iter = cloud_pc->begin(); iter != cloud_pc->end();) {
        auto point = *iter;
        auto pointHeight = point.z;
        //removes if point not in the correct height
        if (pointHeight  < height - threshold || pointHeight > height + threshold) {
                iter = cloud_pc->erase(iter);
        } else {
            iter++;
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloud_pc, cloud_pcl);
    pcl_conversions::fromPCL(cloud_pcl, output);
    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_slicer");

    ros::NodeHandle nh;
    ros::Subscriber velodyne_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_pointcloud", 1, pointcloudCallback);
}

