#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <pluginlib/class_list_macros.h>
#include "laplacian_line_detector.h"

using namespace std;

namespace lane_detector {

    void img_callback(const sensor_msgs::ImageConstPtr& msg) {
        //Convert msg to Mat image
        ros::Time start = ros::Time::now();
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;
        auto time_stamp = msg->header.stamp;

        ros::Time time_convert = ros::Time::now();

        original_height = frame.rows;
        original_width = frame.cols;
        cv::resize(frame, frame, cv::Size(resize_dim * original_width / original_height, resize_dim));

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::Mat second_ignore_color_mask = getIgnoreColorMask(frame, ignore_hsv_range2);
        cv::Mat ignore_color_mask = getIgnoreColorMask(frame, ignore_hsv_range1);
        ros::Time time_resize = ros::Time::now();

        cv::Mat frame_gray = getBlurredGrayImage(frame);

        ros::Time time_color = ros::Time::now();

        cv::Mat lapl, adaptive, true_lines, floodfill_blobs;
        cv::Laplacian(frame_gray, lapl, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
        inRange(lapl, -laplacian_threshold_max, -laplacian_threshold_min, lapl);
        lapl.setTo(cv::Scalar(0), ignore_color_mask | second_ignore_color_mask);

        ros::Time time_lapl = ros::Time::now();
        ros::Time time_adaptive;
        ros::Time time_floodfill;
        if (!ignore_adaptive) {
            adaptive = getAdaptiveThres(frame_gray);
            floodfill_blobs = cutSmall(adaptive, min_blob_area);
            time_adaptive = ros::Time::now();

            cv::Mat fill =  floodfillAreas(lapl, floodfill_blobs);
            true_lines = cutSmall(fill, min_blob_area);
            time_floodfill = ros::Time::now();

        } else {
            cv::Mat lightness;
            threshold(frame_gray, lightness, 5, 255, 0);
            lapl.setTo(cv::Scalar(0, 0, 0), lightness == 0);

            cv::erode(lapl, lapl, kernel(3,1));
            true_lines = cutSmall(lapl, min_blob_area);
            cv::dilate(lapl, lapl, kernel(10,10));

            cv::Mat black(frame.rows,frame.cols,CV_8UC1,cv::Scalar::all(0));
            adaptive = black;
            floodfill_blobs = black;
        }


        cv::Mat img_debug = createDebugImage(frame_gray, adaptive, lapl, floodfill_blobs, ignore_color_mask, second_ignore_color_mask);
        ros::Time time_debug = ros::Time::now();
        cv::resize(true_lines, true_lines, cv::Size(original_width, original_height));
        ros::Time time_done = ros::Time::now();

//        printf("FULL: %f , %Convert: %f , Resize: %f , Color: %f \n" ,
//               (time_debug - start).toSec(), (time_convert - start).toSec(), ;

        printf("FULL: %f , Convert: %f , Resize: %f , Color: %f , Lapl: %f , Adapt: %f , Flood: %f , Debug: %f \n",
                (time_done - start).toSec(), (time_convert - start).toSec(), (time_resize - time_convert).toSec(),
                (time_color - time_resize).toSec(), (time_lapl - time_color).toSec(), (time_adaptive - time_lapl).toSec(),
                (time_floodfill - time_adaptive).toSec(), (time_debug - time_floodfill).toSec());

        publishMessage(pub_line_detector, true_lines, "mono8", time_stamp);
        publishMessage(pub_debug_img, img_debug, "bgr8", time_stamp);
    }


    cv::Mat getIgnoreColorMask(const cv::Mat& frame, color_hsv_range hsv_range) {
        cv::Mat hsv_frame, ignore_color_mask;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_frame,
                cv::Scalar(hsv_range.low_H, hsv_range.low_S, hsv_range.low_V),
                cv::Scalar(hsv_range.high_H, hsv_range.high_S, hsv_range.high_V), ignore_color_mask);

        cv::erode(ignore_color_mask, ignore_color_mask, kernel(2,2));
        return ignore_color_mask;
    }

    cv::Mat getBlurredGrayImage(const cv::Mat& frame) {
        cv::Mat frame_gray, frame_blur;
        cv::GaussianBlur(frame, frame_blur, cv::Size(5,5), 0);
        cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);
        return frame_gray;
    }

    cv::Mat getAdaptiveThres(const cv::Mat& frame_gray) {
        cv::Mat thres;
        cv::adaptiveThreshold(frame_gray, thres, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, -adaptive_mean_threshold);
        blockEnvironment(thres);
        cv::erode(thres, thres, kernel(3,1));
        return thres;
    }

    cv::Mat kernel(int x, int y) {
        return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
    }

    cv::Mat  floodfillAreas(const cv::Mat& lines, const cv::Mat& color_found) {
        cv::Mat color_left, lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
        cv::Mat lines_remaining = lines.clone();
        lines.copyTo(color_left, color_found);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(color_left, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        for(int i = 0; i < contours.size(); i++ ) {
            cv::floodFill(lines_remaining, contours[i][0], cv::Scalar(0));
        }
        cv::bitwise_xor(lines, lines_remaining, lines_found);
        return lines_found;
    }

    void blockEnvironment(const cv::Mat& img) {
        cv::rectangle(img,
                      cv::Point(0,0),
                      cv::Point(img.cols, blockSky_height * resize_dim / original_height),
                      cv::Scalar(0, 0, 0),CV_FILLED);

        cv::rectangle(img,
                      cv::Point(0,img.rows),
                      cv::Point(img.cols, blockWheels_height * resize_dim / original_height),
                      cv::Scalar(0),CV_FILLED);

        cv::rectangle(img,
                      cv::Point(img.cols/3,img.rows),
                      cv::Point(2 * img.cols / 3, blockBumper_height * resize_dim / original_height),
                      cv::Scalar(0),CV_FILLED);
    }

    void blockBottom(const cv::Mat& img) {
        cv::rectangle(img,
                      cv::Point(0,0),
                      cv::Point(img.cols, blockSky_height * resize_dim / original_height - 50),
                      cv::Scalar(0, 0, 0),CV_FILLED);

        cv::rectangle(img,
                      cv::Point(img.cols/3,img.rows),
                      cv::Point(2 * img.cols / 3, blockBumper_height * resize_dim / original_height),
                      cv::Scalar(0),CV_FILLED);

//    cv::fillPoly(img, )
    }

    cv::Mat cutSmall(const cv::Mat& color_edges, int size_min) {
        cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(color_edges, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        for( int i = 0; i < contours.size(); i++ ) {
            if (size_min < cv::contourArea(contours[i], false) ) {
                cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);
            }
        }
        return contours_color;
    }

    void publishMessage(ros::Publisher& pub, const cv::Mat& img, std::string img_type, ros::Time& time_stamp) {
            sensor_msgs::Image outmsg;
            cv_ptr->image = img;
            cv_ptr->encoding = img_type;
            cv_ptr->toImageMsg(outmsg);
            outmsg.header.stamp = time_stamp;
            pub.publish(outmsg);
    }

    cv::Mat overlayBinaryGreen(cv::Mat& frame, const cv::Mat& binary) {
        return frame.setTo(cv::Scalar(0,255,0), binary != 0);
    }

    cv::Mat removeAngels(const cv::Mat& img, int distanceFromEarth) {
        //Removes anything that extends from the top of the image to the bottom like glare from the sun
        cv::Mat top = img.clone();
        cv::rectangle(top,
                      cv::Point(0,img.rows / 3 + blockSky_height - distanceFromEarth),
                      cv::Point(img.cols,img.rows),
                      cv::Scalar(0),CV_FILLED);

        std::vector<cv::Point> locations;
        cv::findNonZero(top, locations);
        int number_of_angles = 20;
        for (int i = 0; i < number_of_angles; ++i) {
            cv::floodFill(img, locations[i], cv::Scalar(0));
            cv::floodFill(top, locations[i], cv::Scalar(0));
            cv::findNonZero(top, locations);
        }
        return img;
    }

    cv::Mat createDebugImage(cv::Mat &img_debug, const cv::Mat &adaptive, const cv::Mat &lapl,
                             const cv::Mat &cut, const cv::Mat& ignore_color, const cv::Mat& ignore_color2) {
        cv::cvtColor(img_debug, img_debug, CV_GRAY2BGR);

        // Highlight ROI
        double alpha = .8;
        cv::Mat img_ROI(img_debug.rows, img_debug.cols, CV_8UC3, cv::Scalar::all(255));
        cv::Mat img_ROI_binary, floodfill_pnt;
        blockEnvironment(img_ROI);
        cv::cvtColor(img_ROI, img_ROI_binary, CV_BGR2GRAY);
        img_ROI.setTo(cv::Scalar(0, 255, 0), img_ROI_binary == 255);
        cv::addWeighted(img_debug, alpha, img_ROI, 1 - alpha, 0.0, img_debug);

        //Add highlighted section colors
        img_debug.setTo(cv::Scalar(0, 0, 130), adaptive != 0);
        img_debug.setTo(cv::Scalar(130, 0, 50), lapl != 0);
        img_debug.setTo(cv::Scalar(204, 0, 204), cut != 0);
        cv::bitwise_and(cut, lapl, floodfill_pnt);
        img_debug.setTo(cv::Scalar(0, 255, 0), floodfill_pnt != 0);
        img_debug.setTo(cv::Scalar(0, 255, 255), ignore_color != 0);
        img_debug.setTo(cv::Scalar(0, 255, 255), ignore_color2 != 0);

        //Add Text
        cv::putText(img_debug, "Area Visible", cv::Point(5,20), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 200, 0), 1);
        cv::putText(img_debug, "Adaptive", cv::Point(5,40), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 0, 130), 1);
        cv::putText(img_debug, "Adaptive (Big Enough)", cv::Point(5,60), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(204, 0, 204), 1);
        cv::putText(img_debug, "Laplacian", cv::Point(5,80), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(130, 0, 50), 1);
        cv::putText(img_debug, "Flood Points (Adpt. && Lapl.)", cv::Point(5,100), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 255, 0), 1);
        cv::putText(img_debug, "Color Being Ignored", cv::Point(5,120), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 255, 255), 1);

        return img_debug;
    }


    void laplacian_line_detector::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        string subscription_node, detection_node, debug_node;
        pnh.param("laplacian_threshold_min", laplacian_threshold_min, 20);
        pnh.param("laplacian_threshold_max", laplacian_threshold_max, 255);
        pnh.param("min_blob_area", min_blob_area, 60);

        pnh.param("blockSky_height",    blockSky_height, 0);
        pnh.param("blockWheels_height", blockWheels_height, 800);
        pnh.param("blockBumper_height", blockBumper_height, 800);

        pnh.param("ignore_adaptive", ignore_adaptive, false);
        pnh.param("adaptive_mean_threshold", adaptive_mean_threshold, 1);


        pnh.param("ignore_color_low_H",  ignore_hsv_range1.low_H,  -1);
        pnh.param("ignore_color_high_H", ignore_hsv_range1.high_H, -1);
        pnh.param("ignore_color_low_S",  ignore_hsv_range1.low_S,  -1);
        pnh.param("ignore_color_high_S", ignore_hsv_range1.high_S, -1);
        pnh.param("ignore_color_low_V",  ignore_hsv_range1.low_V,  -1);
        pnh.param("ignore_color_high_V", ignore_hsv_range1.high_V, -1);

        pnh.param("second_ignore_color_low_H",  ignore_hsv_range2.low_H,  -1);
        pnh.param("second_ignore_color_high_H", ignore_hsv_range2.high_H, -1);
        pnh.param("second_ignore_color_low_S",  ignore_hsv_range2.low_S,  -1);
        pnh.param("second_ignore_color_high_S", ignore_hsv_range2.high_S, -1);
        pnh.param("second_ignore_color_low_V",  ignore_hsv_range2.low_V,  -1);
        pnh.param("second_ignore_color_high_V", ignore_hsv_range2.high_V, -1);

        pnh.param("subscription_node", subscription_node, string("/camera_center/image_color_rect"));
        pnh.param("publish_detection_node", detection_node, string("lines/detection_img"));
        pnh.param("publish_debug_node", debug_node, string("lines/debug_img"));

        img_real = nh.subscribe(subscription_node, 1, img_callback);

        pub_line_detector = nh.advertise<sensor_msgs::Image>(detection_node, 1); //test publish of image
        pub_debug_img = nh.advertise<sensor_msgs::Image>(debug_node, 1);
        ROS_INFO_STREAM("Lane detector ready! Subscribed to " << subscription_node);
        ROS_INFO_STREAM("Publishing to " << debug_node << ", " << detection_node);
    }
}

PLUGINLIB_EXPORT_CLASS(lane_detector::laplacian_line_detector, nodelet::Nodelet)
