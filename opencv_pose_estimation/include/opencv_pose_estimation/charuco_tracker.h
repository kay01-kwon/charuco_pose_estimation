#ifndef CHARUCO_TRACKER_H
#define CHARUCO_TRACKER_H

#include <ros/ros.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Transform.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using sensor_msgs::ImageConstPtr;
using std::vector;
using geometry_msgs::Transform;

using Eigen::Vector3d;
using Eigen::Matrix3d;

using std::cout;
using std::cin;

static const std::string windowName1 = "Gray image";

class CharucoTracker{

    public:
    
    // Constructor
    CharucoTracker();

    // Image Callback function
    void imageCallback(const ImageConstPtr& img_msg);

    // Destructor
    ~CharucoTracker();


    private:
    
    ros::NodeHandle nh;
    
    image_transport::Subscriber image_subscriber;

    ros::Publisher pose_publisher;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;

    cv::Mat image_transport;
    
    cv::Size BoardSize;
    cv::Mat camera_matrix;
    cv::Mat distortionCoeffs;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat tf;
    cv::Mat translation;
    cv::Mat rotation;
    cv::Mat R;
    std::vector<cv::Point3f> corners_info;

    float squareSize;
    Vector3d position;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;

};



#endif