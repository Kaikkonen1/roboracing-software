#include "ros/ros.h"
#include "tag_detection.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_tag_pointcloud");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;
    std::string camera_frame, pointcloud, tag_detections_topic;
    nhp.getParam("camera_frame", camera_frame);
    nhp.getParam("pointcloud", pointcloud);
    nhp.getParam("tag_detections_topic", tag_detections_topic);
    tag_detection tagDetection(&nh, camera_frame, pointcloud, tag_detections_topic);

    ros::spin();
    return 0;
}