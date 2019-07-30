#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <pluginlib/class_list_macros.h>
#include "laplacian_line_detector.h"

using namespace std;

namespace lane_detector {

    double time_sum = 0.0;
    int time_n = 0;

    cv::Mat color_mask, gray_mask;

    void callback(rr_iarrc::laplacian_line_detectorConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request: %d %f %s %s %d",
                config.int_param,
                config.double_param,
                config.str_param.c_str(),
                config.bool_param?"True":"False",
                config.size);
    }

    void img_callback(const sensor_msgs::ImageConstPtr& msg) {
        //Convert msg to Mat image
        ros::Time start = ros::Time::now();
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;
        auto time_stamp = msg->header.stamp;

        orig_size = frame.size();
        hgt_resize_pct = (double) resize_hgt / orig_size.height;
        cv::resize(frame, frame, cv::Size(orig_size.width * hgt_resize_pct, resize_hgt));

        calcRoiRects(frame);

        //Laplacian
        cv::Mat   frame_lapl_ROI = frame(lapl_ROI_rect);
        cv::Mat    gray_lapl_ROI = getBlurredGray(frame_lapl_ROI);
        cv::Mat         lapl_ROI = getLaplacian(gray_lapl_ROI);

        cv::Mat blocked_lapl_ROI = blockColor(frame_lapl_ROI, gray_lapl_ROI, lapl_ROI);

        cv::Mat lapl(frame.size(), CV_8UC1, cv::Scalar::all(0));
        blocked_lapl_ROI.copyTo(lapl(lapl_ROI_rect));
        blockBottom(lapl);

        //Adaptive
        cv::Mat  gray_adapt_ROI = gray_lapl_ROI(rel_adapt_ROI_rect);
        cv::Mat adapt_adapt_ROI = getAdaptiveThres(gray_adapt_ROI);
        cv::Mat blobs_adapt_ROI = cutSmall(adapt_adapt_ROI);

        cv::Mat adapt(frame.size(), CV_8UC1, cv::Scalar::all(0));
        blobs_adapt_ROI.copyTo(adapt(adapt_ROI_rect));

        //Floodfill
        cv::Mat true_lines = floodfillAreas(lapl, adapt);

        cv::Mat img_debug = createDebugImage(frame, lapl, adapt, blobs_adapt_ROI, gray_mask);

        cv::resize(true_lines, true_lines, orig_size);

        ros::Time time_done = ros::Time::now();
        time_sum += (time_done - start).toSec();
        time_n++;
        printf("FULL: %f , TimeAvg: %f \n", (time_done - start).toSec(), (time_sum / time_n));

        publishMessage(pub_line_detector, true_lines, "mono8", time_stamp);
        publishMessage(pub_debug_img, img_debug, "bgr8", time_stamp);
    }

    void calcRoiRects(const cv::Mat& frame) {
        lapl_ROI_rect = cv::Rect(cv::Point(0,           lapl_ROI_top * hgt_resize_pct),
                                 cv::Point(frame.cols,  lapl_ROI_btm * hgt_resize_pct));

        rel_adapt_ROI_rect = cv::Rect(cv::Point(0,          (adapt_ROI_top - lapl_ROI_top) * hgt_resize_pct),
                                      cv::Point(frame.cols, (adapt_ROI_btm - lapl_ROI_top) * hgt_resize_pct));

        adapt_ROI_rect = cv::Rect(0, adapt_ROI_top * hgt_resize_pct, frame.cols, rel_adapt_ROI_rect.height);
    }

    cv::Mat getBlurredGray(const cv::Mat& frame) {
        cv::Mat frame_gray, frame_blur;
        cv::GaussianBlur(frame, frame_blur, cv::Size(5,5), 0);
        cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);
        return frame_gray;
    }

    cv::Mat getLaplacian(const cv::Mat& gray) {
        cv::Mat lapl_ROI;
        cv::Laplacian(gray, lapl_ROI, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
        cv::inRange(lapl_ROI, -laplacian_threshold_max, -laplacian_threshold_min, lapl_ROI);
        return lapl_ROI;
    }

    cv::Mat blockColor(const cv::Mat& frame, const cv::Mat& gray, cv::Mat& lapl) {
        cv::Mat hsv_frame, color1_mask, color2_mask, comb_mask;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        cv::inRange(hsv_frame, color1_hsv_low, color1_hsv_high, color1_mask);
        cv::inRange(hsv_frame, color2_hsv_low, color2_hsv_high, color2_mask);
        cv::bitwise_or(color1_mask, color2_mask, color_mask);
        cv::threshold(gray, gray_mask, gray_threshold, 255, 1);

        cv::dilate(color_mask, color_mask, kernel(3,3));
        cv::dilate(gray_mask, gray_mask, kernel(5,5));

        cv::bitwise_or( color_mask, gray_mask, comb_mask);
        cv::bitwise_not(comb_mask, comb_mask);
        cv::bitwise_and(lapl, comb_mask, lapl);

        return lapl;
    }

    void blockBottom(const cv::Mat& img) {
        cv::rectangle(img,
                      cv::Point(img.cols/3,img.rows),
                      cv::Point(2 * img.cols / 3, blockBumper_top * hgt_resize_pct),
                      cv::Scalar(0), CV_FILLED);
    }

    cv::Mat getAdaptiveThres(const cv::Mat& gray) {
        cv::Mat thres;
        cv::adaptiveThreshold(gray, thres, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, -adaptive_mean_threshold);
        cv::erode(thres, thres, kernel(4,1));
        return thres;
    }

    cv::Mat cutSmall(const cv::Mat& color_edges) {
        cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(color_edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        for( int i = 0; i < contours.size(); i++ ) {
            if (min_blob_area < cv::contourArea(contours[i], false) ) {
                cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);
            }
        }
        return contours_color;
    }

    cv::Mat floodfillAreas(const cv::Mat& noisy, const cv::Mat& precise) {
        cv::Mat fill_here, filled_here, mask;
        cv::Mat blank(noisy.rows, noisy.cols, CV_8UC1, cv::Scalar::all(0));
        cv::bitwise_and(noisy, precise, fill_here);

        cv::copyMakeBorder(noisy, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);
        cv::bitwise_not(mask, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fill_here, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        for(int i = 0; i < contours.size(); i++) {
            cv::floodFill(blank, mask, contours[i][0], cv::Scalar(255));
        }
        return blank;
    }

    cv::Mat kernel(int x, int y) {
        return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
    }

    cv::Mat createDebugImage(cv::Mat &img_debug, const cv::Mat &lapl, const cv::Mat &adapt,
                             const cv::Mat &blobs_adapt_ROI, const cv::Mat &gray_block) {
        if (pub_debug_img.getNumSubscribers() > 0) {
            //Get 3-channel gray img
            cv::cvtColor(img_debug, img_debug, CV_BGR2GRAY);
            cv::cvtColor(img_debug, img_debug, CV_GRAY2BGR);

            // Highlight ROI
            double alpha = .8;
            cv::Mat lapl_ROI_frame(img_debug.size(), CV_8UC3, cv::Scalar::all(0));
            lapl_ROI_frame(lapl_ROI_rect).setTo(cv::Scalar(255, 0, 0));
            cv::addWeighted(img_debug, alpha, lapl_ROI_frame, 1 - alpha, 0.0, img_debug);

            cv::Mat adapt_ROI_frame(img_debug.size(), CV_8UC3, cv::Scalar::all(0));
            adapt_ROI_frame(adapt_ROI_rect).setTo(cv::Scalar(0, 255, 0));
            cv::addWeighted(img_debug, alpha, adapt_ROI_frame, 1 - alpha, 0.0, img_debug);

            //Thresholding
            cv::Mat floodfill_pnt;
            img_debug.setTo(cv::Scalar(255, 255, 255), lapl);
            img_debug.setTo(cv::Scalar(204, 0, 204), adapt);
            cv::bitwise_and(lapl, adapt, floodfill_pnt);
            img_debug.setTo(cv::Scalar(0, 255, 0), floodfill_pnt);


            //Highlight Blocked
            cv::Mat gray_thres_frame(img_debug.size(), CV_8UC1, cv::Scalar::all(0));
            gray_mask.copyTo(gray_thres_frame(lapl_ROI_rect));
            img_debug.setTo(cv::Scalar(70, 70, 70), gray_thres_frame);

            cv::Mat color_thres_frame(img_debug.size(), CV_8UC1, cv::Scalar::all(0));
            color_mask.copyTo(color_thres_frame(lapl_ROI_rect));
            img_debug.setTo(cv::Scalar(0, 200, 200), color_thres_frame);

//            //Add Text
//            cv::putText(img_debug, "Area Visible", cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 200, 0),
//                        1);
//            cv::putText(img_debug, "Adaptive", cv::Point(5, 40), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 0, 130), 1);
//            cv::putText(img_debug, "Adaptive (Big Enough)", cv::Point(5, 60), cv::FONT_HERSHEY_DUPLEX, .7,
//                        cv::Scalar(204, 0, 204), 1);
//            cv::putText(img_debug, "Laplacian", cv::Point(5, 80), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(130, 0, 50),
//                        1);
//            cv::putText(img_debug, "Flood Points (Adpt. && Lapl.)", cv::Point(5, 100), cv::FONT_HERSHEY_DUPLEX, .7,
//                        cv::Scalar(0, 255, 0), 1);
//            cv::putText(img_debug, "Color Being Ignored", cv::Point(5, 120), cv::FONT_HERSHEY_DUPLEX, .7,
//                        cv::Scalar(0, 255, 255), 1);
        }
        return img_debug;
    }

    cv::Mat overlayBinaryGreen(cv::Mat& frame, const cv::Mat& binary) {
        return frame.setTo(cv::Scalar(0,255,0), binary != 0);
    }

    void publishMessage(ros::Publisher& pub, const cv::Mat& img, std::string img_type, ros::Time& time_stamp) {
        if (pub.getNumSubscribers() > 0) {
            sensor_msgs::Image outmsg;
            cv_ptr->image = img;
            cv_ptr->encoding = img_type;
            cv_ptr->toImageMsg(outmsg);
            pub.publish(outmsg);
        }
    }


    void laplacian_line_detector::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        pnh.param("laplacian_threshold_min", laplacian_threshold_min, 20);
        pnh.param("laplacian_threshold_max", laplacian_threshold_max, 255);

        pnh.param("adaptive_mean_threshold", adaptive_mean_threshold, 0);
        pnh.param("min_blob_area", min_blob_area, 20);
        pnh.param("ignore_adaptive", ignore_adaptive, false);

        pnh.param("lapl_ROI_top",    lapl_ROI_top, 500);
        pnh.param("lapl_ROI_bottom", lapl_ROI_btm, 860);

        pnh.param("adapt_ROI_top", adapt_ROI_top, 550);
        pnh.param("adapt_ROI_bottom", adapt_ROI_btm, 750);

        pnh.param("blockBumper_top", blockBumper_top, 800);

        int H_low, S_low, V_low, H_high, S_high, V_high;
        pnh.param("ignore_color1_H_low",  H_low,  -1);
        pnh.param("ignore_color1_S_low",  S_low,  -1);
        pnh.param("ignore_color1_V_low",  V_low,  -1);
        pnh.param("ignore_color1_H_high", H_high, -1);
        pnh.param("ignore_color1_S_high", S_high, -1);
        pnh.param("ignore_color1_V_high", V_high, -1);
        color1_hsv_low = cv::Scalar( H_low , S_low , V_low );
        color1_hsv_high = cv::Scalar(H_high, S_high, V_high);

        pnh.param("ignore_color2_H_low",  H_low,  -1);
        pnh.param("ignore_color2_S_low",  S_low,  -1);
        pnh.param("ignore_color2_V_low",  V_low,  -1);
        pnh.param("ignore_color2_H_high", H_high, -1);
        pnh.param("ignore_color2_S_high", S_high, -1);
        pnh.param("ignore_color2_V_high", V_high, -1);
        color2_hsv_low = cv::Scalar( H_low , S_low , V_low );
        color2_hsv_high = cv::Scalar(H_high, S_high, V_high);

        pnh.param("ignore_gray_threshold", gray_threshold, -1);

        string subscription_node, detection_node, debug_node;
        pnh.param("subscription_node", subscription_node, string("/camera_center/image_color_rect"));
        pnh.param("publish_detection_node", detection_node, string("lines/detection_img"));
        pnh.param("publish_debug_node", debug_node, string("lines/debug_img"));

        img_real = nh.subscribe(subscription_node, 1, img_callback);

        pub_line_detector = nh.advertise<sensor_msgs::Image>(detection_node, 1); //test publish of image
        pub_debug_img = nh.advertise<sensor_msgs::Image>(debug_node, 1);

        dynamic_reconfigure::Server<rr_iarrc::laplacian_line_detectorConfig> server;
        dynamic_reconfigure::Server<rr_iarrc::laplacian_line_detectorConfig>::CallbackType f;

        f = boost::bind(&callback, _1, _2);
        server.setCallback(f);

        ROS_INFO_STREAM("Lane detector ready! Subscribed to " << subscription_node);
        ROS_INFO_STREAM("Publishing to " << debug_node << ", " << detection_node);
        ros::spin();
    }
}

PLUGINLIB_EXPORT_CLASS(lane_detector::laplacian_line_detector, nodelet::Nodelet)
