#include "chessboard_tracker.h"

ChessBoardTracker::ChessBoardTracker()
{
    nh.getParam("SqaureSize",squareSize);
    nh.getParam("BoardWidth",BoardSize.width);
    nh.getParam("BoardHeight",BoardSize.height);

    camera_matrix = (cv::Mat1d(3,3) << 674.044814, 0, 330.513334,
                                        0, 672.730994, 231.698965,
                                        0, 0, 1);
    
    distortionCoeffs = (cv::Mat1d(1,5) << 0.160051,
                                        -0.290046,
                                        -0.000658,
                                        0.001140,
                                        0);

    std::cout<<"Camera Matrix \n";
    std::cout<<camera_matrix<<std::endl;

    std::cout<<"Distortion Coefficients\n";
    std::cout<<distortionCoeffs<<std::endl;

    //BoardSize.width = 7;
    //BoardSize.height = 5;
    squareSize = 0.029;

    for(int i = 0; i < BoardSize.height; i++ )
        for(int j = 0; j < BoardSize.width; j++ )
            corners_info.push_back(cv::Point3f(float(j*squareSize),
                                                float(i*squareSize),
                                                0));

    //std::cout<<corners_info<<std::endl;
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("/camera/color/image_raw",1,&ChessBoardTracker::imageCallback,this);
    pose_publisher = nh.advertise<Transform>("/pose",1);

}

void ChessBoardTracker::imageCallback(const ImageConstPtr& img_msg)
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
    std::vector<cv::Point2f> detected_corners;

//    cv::imshow("original image",img);

    bool found = cv::findChessboardCorners(cv_ptr->image,BoardSize,detected_corners);

//    cv::imshow("image",img);


    if(found == false){
        std::cout<<"Not detected\n";
        return;
    }

    

    cv::drawChessboardCorners(img, BoardSize,detected_corners,found);
    cv::solvePnP(corners_info,
    detected_corners,
    camera_matrix,
    distortionCoeffs,
    rvec,tvec);

    double *position = (double*) translation.data;

    //std::cout<<position[0]<<std::endl;
    
    std::cout<<tvec<<std::endl;
    std::cout<<"\n";



    std::vector<cv::Point3f> obj_pts;
    obj_pts.push_back(cv::Point3f(0,0,0));
    obj_pts.push_back(cv::Point3f(0.066,0,0));
    obj_pts.push_back(cv::Point3f(0,0.066,0));
    obj_pts.push_back(cv::Point3f(0,0,0.066));

    cv::Rodrigues(rvec,R);
    rotation = R;

    std::vector<cv::Point2f> imagePoints;

    cv::projectPoints(obj_pts,rvec,tvec,
    camera_matrix,distortionCoeffs,imagePoints);

    cv::line(img,imagePoints.at(0),imagePoints.at(1),cv::Scalar(0,0,255),5,8,0);
    cv::line(img,imagePoints.at(0),imagePoints.at(2),cv::Scalar(0,255,0),5,8,0);
    cv::line(img,imagePoints.at(0),imagePoints.at(3),cv::Scalar(255,0,0),5,8,0);

    cv::imshow("detected coordinate",img);

    cv::waitKey(1);

}

void ChessBoardTracker::GetPose()
{
    
}

ChessBoardTracker::~ChessBoardTracker()
{

}