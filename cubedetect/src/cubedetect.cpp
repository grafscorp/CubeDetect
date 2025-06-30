#include <cubedetect.hpp>

double lineAngleDegrees(const cv::Vec4i &line)
{
    double dx = line[2] - line[0];
    double dy = line[3] - line[1];
    double angle = atan2(dy,dx) * 180.0 / CV_PI; //radian angle to degrees

    return (angle < 0) ? angle + 180: angle; //normolize angle in [0,180]
}


cv::Point2f linesIntersection(const cv::Vec4i &line1, const cv::Vec4i &line2)
{
    //Get coords ends of the lines
    float x1 = line1[0], y1 = line1[1];
    float x2 = line1[2], y2 = line1[3];
    float x3 = line2[0], y3 = line2[1];
    float x4 = line2[2], y4 = line2[3];

    //Denominator for equation lines
    float denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);

    /*If lines is parallel 
    @var denom is close to zero
    */
    if(fabs(denom) < 1e-5) return cv::Point2f(-1,-1);

    float ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;

    return cv::Point2f(
        x1 + ua*(x2-x1),
        y1+ua*(y2-y1)
    );



}

