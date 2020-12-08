//
// Created by Charles Jenkins on 11/27/20.
//

#pragma once

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <utility>

class tag_detection {
  public:
    tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud,
                  const std::string &tag_detections_topic, std::string destination_frame, double x_offset,
                  double y_offset, double px_per_m, double width, double height);

  private:
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_markers;
    ros::Subscriber sub_detections;
    tf2_ros::Buffer tfBuffer;
    tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
    std::string camera_frame, destination_frame;
    double x_offset, y_offset, px_per_m, width, height;

    tf::Pose draw_opponent(geometry_msgs::Pose april_tag_center);

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    void create_marker(const int &id, geometry_msgs::Pose april_w);
};