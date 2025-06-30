#include <cubedetect.hpp>

namespace cubedetectcv{

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

void extendLinesToIntersections(std::vector<cv::Vec4i>& lines, const std::vector<cv::Point2f>& intersections, float maxDist) {
    for (auto& line : lines) {
        // Получаем конечные точки текущей линии
        cv::Point2f p1(line[0], line[1]);
        cv::Point2f p2(line[2], line[3]);
        
        cv::Point2f bestP1 = p1;  // Ближайшая точка к p1
        cv::Point2f bestP2 = p2;  // Ближайшая точка к p2
        float bestDist1 = maxDist;  // Лучшее расстояние для p1
        float bestDist2 = maxDist;  // Лучшее расстояние для p2
        
        // Поиск ближайших вершин к концам линии
        for (const auto& pt : intersections) {
            float d1 = norm(p1 - pt);  // Расстояние от p1 до вершины
            float d2 = norm(p2 - pt);  // Расстояние от p2 до вершины
            
            // Обновляем лучшую точку для p1
            if (d1 < bestDist1) {
                bestDist1 = d1;
                bestP1 = pt;
            }
            
            // Обновляем лучшую точку для p2
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestP2 = pt;
            }
        }
        
        // Обновляем линию, если нашли близкие вершины
        if (bestDist1 < maxDist) {
            line[0] = bestP1.x;
            line[1] = bestP1.y;
        }
        if (bestDist2 < maxDist) {
            line[2] = bestP2.x;
            line[3] = bestP2.y;
        }

        // Проверка минимальной длины линии после обновления
        cv::Vec4i newLine = {line[0], line[1], line[2], line[3]};
        float length = cv::norm(cv::Point2f(newLine[0], newLine[1]) - cv::Point2f(newLine[2], newLine[3]));
        if (length < 10) {  // Если линия стала слишком короткой
            // Восстанавливаем оригинальную линию
            line = {static_cast<int>(p1.x), static_cast<int>(p1.y), 
                    static_cast<int>(p2.x), static_cast<int>(p2.y)};
        }
    }
}



cv::Mat detectAndDrawCube(cv::Mat& srcImage, bool vis_lines, bool vis_pointers, bool vis_vertex , bool vis_conture)
{
    #ifdef DEBUG
    std::cout << "[v] Start proccesing image\n";
    #endif
    //Image preprocessing to optimize subsequent operations
    #pragma region Preproccesing
    cv::Mat cubeImage = srcImage.clone();
    cv::Mat gray, blurred, edges;

    // Конвертация в градации серого
    cvtColor(srcImage, gray, cv::COLOR_BGR2GRAY);
    
    // Bыравнивание гистограммы для компенсации теней
    cv::Mat claheImg;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, claheImg);
    
    // Гауссово размытие для уменьшения шума
    GaussianBlur(claheImg, blurred, cv::Size(5, 5), 1.5);
    
    // Адаптивное пороговое преобразование 
    cv::Mat thresh;
    adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                     cv::THRESH_BINARY, 11, 2);

    Canny(thresh, edges, THRESHOLD1_CANNY, THRESHOLD2_CANNY);

    // Морфологические операции для улучшения линий
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    dilate(edges, edges, kernel_dilate);
    
    cv::Mat kernel_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    erode(edges, edges, kernel_erode);
    
    #pragma endregion Preproccesing


    //Tracing the contours of an image with lines
    #pragma region ComputeLines
    #ifdef DEBUG
    std::cout << "[v] Start compute line\n";
    #endif
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines,
        RHO_HOUGHLINES, THETA_HOUGHLINES, THRESHOLD_HOUGHLINES, 
        MINLINELENGTH_HOUGHLINES, MAXLINEGAP_HOUGHLINES);  
    

    std::vector<cv::Vec4i> filteredLines;
    for (const auto& line : lines) {
        double length = cv::norm(cv::Point2f(line[0], line[1]) - cv::Point2f(line[2], line[3]));
       
        if (length >  FILTRED_LENGHTLINE) {
            filteredLines.push_back(line);
        }
    }



    std::vector<double> angles;
    for (const auto& line : filteredLines) {
        // Вычисляем угол для каждой линии
        angles.push_back(lineAngleDegrees(line));
    }
        #ifdef DEBUG
    std::cout << "[v] line angle degress is completed\n";
    #endif
    #pragma endregion ComputeLines
    
    #pragma region Clusterring
    // Ожидаем 3 основных направления для куба
    cv::Mat anglesMat(angles.size(), 1, CV_32F);
    for (size_t i = 0; i < angles.size(); i++) {
        anglesMat.at<float>(i) = static_cast<float>(angles[i]);
    }

    cv::Mat labels, centers;
    // Применяем алгоритм k-means для кластеризации линий
    kmeans(anglesMat, CLUSTERCOUNT, labels, 
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
            3, cv::KMEANS_PP_CENTERS, centers);

    std::vector<cv::Point2f> intersections;
    for (size_t i = 0; i < filteredLines.size(); i++) {
        for (size_t j = i + 1; j < filteredLines.size(); j++) {
            if (labels.at<int>(i) != labels.at<int>(j)) {
                cv::Point2f pt = linesIntersection(filteredLines[i], filteredLines[j]);
                
                if (pt.x >= 0 && pt.y >= 0 && pt.x < srcImage.cols && pt.y < srcImage.rows) {
                    double dist1 = std::min(norm(pt - cv::Point2f(filteredLines[i][0], filteredLines[i][1])), 
                                    cv::norm(pt - cv::Point2f(filteredLines[i][2], filteredLines[i][3])));
                    double dist2 = std::min(cv::norm(pt - cv::Point2f(filteredLines[j][0], filteredLines[j][1])), 
                                    cv::norm(pt - cv::Point2f(filteredLines[j][2], filteredLines[j][3])));
                    
                    // Допуск для теневых областей
                    if (dist1 < 45 || dist2 < 45) {
                        intersections.push_back(pt);
                    }
                }
            }
        }
    }
        #ifdef DEBUG
    std::cout << "[v] lines Intersection is completed\n";
    #endif

    
    std::vector<cv::Point2f> vertices = clusterPoints(intersections, 25.0f);  
        #ifdef DEBUG
    std::cout << "[v] clustering is completed\n";
    #endif

    extendLinesToIntersections(filteredLines, vertices, 50.0f);  
            #ifdef DEBUG
        std::cout << "[v] extendLinesToIntersections is completed\n";
        #endif
    #pragma endregion 

    #pragma region Vertex
            #ifdef DEBUG
    std::cout << "[v] Start compute vertex\n";
    #endif
    std::vector<cv::Point2f> filteredVertices;
    for (const auto& vertex : vertices) {
        int connections = 0;
        
        for (const auto& line : filteredLines) {
            double dist = distanceToLine(vertex, line);
            // Допуск расстояния
            if (dist < 15.0) {
                connections++;
            }
        }
        
        // Порог соединений для теневых областей
        if (connections >= 1) {  
            filteredVertices.push_back(vertex);
        }
    }
        #ifdef DEBUG
    std::cout << "[v] Distance to line is completed\n";
    #endif
    #pragma endregion

    #pragma region Countour
            #ifdef DEBUG
    std::cout << "[v] Start compute countour\n";
    #endif
    std::vector<cv::Point> cubeContour;
    if (filteredVertices.size() >= 4) {
        std::vector<cv::Point2f> hullPoints;
        convexHull(filteredVertices, hullPoints);
        
        // аппроксимация контура
        std::vector<cv::Point2f> approxHull;
        approxPolyDP(hullPoints, approxHull, 8.0, true); 
        
        // Проверка и коррекция для неполных кубов
        if (approxHull.size() < 4) {
            // Если вершин меньше 4, используем все обнаруженные точки
            for (const auto& pt : filteredVertices) {
                cubeContour.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
            }
        } else {
            for (const auto& pt : approxHull) {
                cubeContour.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
            }
        }
    }
    #pragma endregion 
    
    #pragma region Visualization
            #ifdef DEBUG
    std::cout << "[v] Start compute visualization\n";
    #endif
    // if(vis_lines)
    // {
    //     for (const auto& line : filteredLines) {
    //         cv::line(cubeImage, cv::Point(line[0], line[1]), 
    //                 cv::Point(line[2], line[3]), cv::Scalar(0, 180, 0), 2);
    // }}
    // if(vis_pointers)
    // {
    //     for (const auto& pt : intersections) {
    //         cv::circle(cubeImage, pt, 3, cv::Scalar(255, 0, 0), -1);
    // }}

    // if(vis_vertex)
    // {
    // for (const auto& vertex : filteredVertices) {
    //     cv::circle(cubeImage, vertex, 6, cv::Scalar(0, 0, 255), -1);
    // }}

    if(vis_conture){
        if (!cubeContour.empty()) {
            // Рисуем линии контура
            for (size_t i = 0; i < cubeContour.size(); i++) {
                cv::line(cubeImage, cubeContour[i], cubeContour[(i+1) % cubeContour.size()], 
                        cv::Scalar(255, 0, 255), 3);
            }
            
            // Добавляем прозрачную заливку
            cv::Mat overlay = cubeImage.clone();
            cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{cubeContour}, cv::Scalar(255, 0, 255));
            cv::addWeighted(overlay, 0.2, cubeImage, 0.8, 0, cubeImage);
        }
        }
    #pragma endregion
            #ifdef DEBUG
    std::cout << "[v] Complete detect Cube\n";
    #endif
    return cubeImage;
    }
    
}
