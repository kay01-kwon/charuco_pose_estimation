#include <opencv_pose_estimation/charuco_tracker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "charuco_tracker");

    CharucoTracker pose_estimator;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}