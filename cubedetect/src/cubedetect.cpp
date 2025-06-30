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

std::vector<cv::Point2f> clusterPoints(const std::vector<cv::Point2f>& points, const float tolerance) {
    // Result of cluster
    std::vector<cv::Point2f> clustered;  
    // Visited pointers
    std::vector<bool> visited(points.size(), false);  
    
    for (size_t i = 0; i < points.size(); i++) {
        if (visited[i]) continue;  // Skip processed pointers
        
        cv::Point2f sum = points[i];  // Start new cluster
        int count = 1;
        visited[i] = true;
        
        // Find points 
        for (size_t j = i + 1; j < points.size(); j++) {
            if (visited[j]) continue;
            
            // Check lenght between pointers
            if (norm(points[i] - points[j]) < tolerance) {
                sum += points[j];  // Add point to cluster
                count++;
                visited[j] = true;
            }
        }
        
        // Добавляем центроид кластера в результат
        clustered.push_back(sum / count);
    }

    return clustered;
}

double distanceToLine(const cv::Point2f& point, const cv::Vec4i& line) {
    // Convert line to 2 point
    cv::Point2f linePt1(line[0], line[1]);
    cv::Point2f linePt2(line[2], line[3]);
    
    // Compute vector 
    double dx = linePt2.x - linePt1.x;
    double dy = linePt2.y - linePt1.y;
    
    // Compute lenght line
    double length = sqrt(dx*dx + dy*dy);
    if (length < 1e-5) return norm(point - linePt1);  // Если линия - точка
    
    // Compute param t projection pointer on line
    double t = std::max(0.0, std::min(1.0, ((point.x - linePt1.x)*dx + (point.y - linePt1.y)*dy) / (length*length)));
    
    // Point projection
    cv::Point2f projection(linePt1.x + t * dx, linePt1.y + t * dy);

    // Path from point to projection
    return cv::norm(point - projection);
}

