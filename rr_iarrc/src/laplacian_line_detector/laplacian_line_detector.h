#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <rr_iarrc/laplacian_line_detectorConfig.h>

namespace lane_detector {

    void dynamic_callback(rr_iarrc::laplacian_line_detectorConfig&, uint32_t);
    void img_callback(const sensor_msgs::ImageConstPtr&);
    void calcRoiRects(const cv::Mat&);
    cv::Mat getBlurredGray(const cv::Mat&);
    cv::Mat getLaplacian(const cv::Mat&);
    cv::Mat blockColor(const cv::Mat&, const cv::Mat&, cv::Mat&);
    cv::Mat getAdaptiveThres(const cv::Mat&);
    cv::Mat cutSmall(const cv::Mat&);
    void blockBottom(const cv::Mat&);
    cv::Mat floodfillAreas(const cv::Mat&, const cv::Mat&);
    cv::Mat createDebugImage(cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&);
    cv::Mat overlayBinaryGreen(cv::Mat&, const cv::Mat&);
    cv::Mat kernel(int, int);
    void publishMessage(ros::Publisher&, const cv::Mat&, std::string, ros::Time&);

    cv::Size orig_size;
    int resize_hgt = 400;
    double hgt_resize_pct;
    cv::Rect lapl_ROI_rect, rel_adapt_ROI_rect, adapt_ROI_rect;
    cv::Mat color_mask, gray_mask;

    int laplacian_threshold_min, laplacian_threshold_max, adaptive_mean_threshold, min_blob_area;
    bool ignore_adaptive;

    int lapl_ROI_top, lapl_ROI_btm, adapt_ROI_top, adapt_ROI_btm, blockBumper_top;

    cv::Scalar color1_hsv_low, color1_hsv_high, color2_hsv_low, color2_hsv_high;
    int gray_threshold;

    cv_bridge::CvImagePtr cv_ptr;
    ros::Subscriber img_real;
    ros::Publisher pub_line_detector, pub_debug_img;

    class laplacian_line_detector : public nodelet::Nodelet {
        private:
            virtual void onInit();
    };
}