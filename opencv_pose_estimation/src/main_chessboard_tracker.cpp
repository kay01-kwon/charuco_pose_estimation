#include <opencv_pose_estimation/chessboard_tracker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chessboard_tracker");

    ChessBoardTracker pose_estimator;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}