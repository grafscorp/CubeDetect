#include <cubedetect.hpp>

double lineAngleDegrees(const cv::Vec4i &line)
{
    double dx = line[2] - line[0];
    double dy = line[3] - line[1];
    double angle = atan2(dy,dx) * 180.0 / CV_PI; //radian angle to degrees

    return (angle < 0) ? angle + 180: angle; //normolize angle in [0,180]
}


