#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace lane_detector {

    struct color_hsv_range {
        int low_H;
        int high_H;
        int low_S;
        int high_S;
        int low_V;
        int high_V;
    };

    void img_callback(const sensor_msgs::ImageConstPtr&);
    void blockEnvironment(const cv::Mat&);
    cv::Mat getAdaptiveThres(const cv::Mat&);
    void blockBottom(const cv::Mat&);
    cv::Mat getIgnoreColorMask(const cv::Mat&,  color_hsv_range);
    cv::Mat getBlurredGrayImage(const cv::Mat&);
    cv::Mat kernel(int, int);
    cv::Mat floodfillAreas(const cv::Mat&, const cv::Mat&);
    cv::Mat cutSmall(const cv::Mat&, int);
    void publishMessage(ros::Publisher&, const cv::Mat&, std::string, ros::Time&);
    cv::Mat overlayBinaryGreen(cv::Mat&, const cv::Mat&);
    cv::Mat removeAngels(const cv::Mat&, int);

    cv::Mat createDebugImage(cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&);

    color_hsv_range ignore_hsv_range1;
    color_hsv_range ignore_hsv_range2;

    int original_width, original_height;

    int resize_dim = 400;
    bool ignore_adaptive;

    int min_blob_area, laplacian_threshold_min, laplacian_threshold_max, adaptive_mean_threshold;
    int blockSky_height, blockWheels_height, blockBumper_height;

    ros::Subscriber img_real;
    cv_bridge::CvImagePtr cv_ptr;
    ros::Publisher pub_line_detector, pub_debug_img;

    class laplacian_line_detector : public nodelet::Nodelet {
        private:
            virtual void onInit();
    };
}