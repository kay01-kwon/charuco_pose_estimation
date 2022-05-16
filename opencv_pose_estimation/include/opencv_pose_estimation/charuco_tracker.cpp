#include "charuco_tracker.h"

CharucoTracker::CharucoTracker()
{
    //nh.getParam("BoardWidth",BoardSize.width);
    //nh.getParam("BoardHeight",BoardSize.height);

    BoardSize.width = 5;
    BoardSize.height = 7;

    camera_matrix = (cv::Mat1d(3,3) << 674.044814, 0, 330.513334,
                                        0, 672.730994, 231.698965,
                                        0, 0, 1);
    
    distortionCoeffs = (cv::Mat1d(1,5) << 0.160051,
                                        -0.290046,
                                        -0.000658,
                                        0.001140,
                                        0);

    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("/camera/color/image_raw",1,&CharucoTracker::imageCallback,this);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    board = cv::aruco::CharucoBoard::create(BoardSize.width,BoardSize.height,0.037,0.022,dictionary);
    params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

}

void CharucoTracker::imageCallback(const ImageConstPtr& img_msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv bridge exception: %s",e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image,img,cv::IMREAD_COLOR);
    cv::imshow("Original Image",img);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;


    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, params);


    if(markerIds.size() == 0)
        std::cout<<"Not detected \n";

    if(markerIds.size()>0)
    {

        cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
            cv::aruco::drawDetectedCornersCharuco(img, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
        
        bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix, distortionCoeffs, rvec, tvec);
        // if charuco pose is valid
        if (valid){
            cv::aruco::drawAxis(img, camera_matrix, distortionCoeffs, rvec, tvec, 0.1f);
            
            double *p = (double*) tvec.data;
            std::cout<<"x: ";
            std::cout<<p[0]<<"\t";
            std::cout<<"y: ";
            std::cout<<p[1]<<"\t";
            std::cout<<"z: ";
            std::cout<<p[2]<<std::endl;
        }
    }

    cv::imshow("Detected Image",img);

    cv::waitKey(1);

}

CharucoTracker::~CharucoTracker()
{

}