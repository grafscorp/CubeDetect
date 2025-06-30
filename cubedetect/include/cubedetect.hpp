#ifndef CUBEDETECT_HPP
#define CUBEDETECT_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
//#define DEBUG
namespace cubedetectcv{
//Consts for proccesing image algorimts 
const double THRESHOLD1_CANNY = 30.0, THRESHOLD2_CANNY = 100.0;
const double RHO_HOUGHLINES =1 , THETA_HOUGHLINES = CV_PI/180, MINLINELENGTH_HOUGHLINES = 25,  MAXLINEGAP_HOUGHLINES = 15;
const int THRESHOLD_HOUGHLINES=35;
const double FILTRED_LENGHTLINE = 20.0;
const int CLUSTERCOUNT = 3;
//Compute angles line to degrees
double lineAngleDegrees(const cv::Vec4i &line);

//Compute point for intersections two lines
cv::Point2f linesIntersection(const cv::Vec4i &line1, const cv::Vec4i &line2);

/* Combining points(Clustter)
@param point
@param lerance 
*/
std::vector<cv::Point2f> clusterPoints(const std::vector<cv::Point2f>& points, const float tolerance = 15.0f);

// Функция для вычисления расстояния от точки до линии
double distanceToLine(const cv::Point2f& point, const cv::Vec4i& line);

// Функция для продления линий до ближайших вершин
void extendLinesToIntersections(std::vector<cv::Vec4i>& lines, const std::vector<cv::Point2f>& intersections, float maxDist = 25.0f);

/*Detect cube in srcImage, and returned mat with hilighted cube
@param srcImage  source image
@param vis_lines visualiation cube lines
@param vis_pointer visualiation intersection pointers 
@param vis_vertex visualiation cube vertex
@param vis_conture visualiation cube conture

*/
cv::Mat detectAndDrawCube(cv::Mat& srcImage, bool vis_lines = false, bool vis_pointers = false, bool vis_vertex = false , bool vis_conture = true);
}

#endif