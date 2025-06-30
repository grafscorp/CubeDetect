
#include <cubedetect.hpp>
#include <string>
#include <iostream>

void showVideoCube(std::string filename);
void showImageCube(std::string filename);

int main(int argc, char const *argv[])
{
    #ifdef DEBUG
    std::cout << "Start\n";
    #endif
    std::string filename;
    if(argc > 1)
    {  
        filename = argv[1];
    }
    else{
        filename = "cube.jpg";
    }
    #ifdef DEBUG
    std::cout << "Defining the file type : "<< filename << std::endl;
    #endif

    if(filename.substr(filename.find_last_of(".")+1) == "mp4" )
    {
        //Video
        try
        {
            showVideoCube(filename);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return -1;
        }
        
    }
    else{
        //Image
        try
        {
            showImageCube(filename);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return -1;
        }
        
       
    }
    #ifdef DEBUG
    std::cout << "End\n";
    #endif
     #ifdef DEBUG
    std::cout << "\n";
    #endif
        cv::VideoCapture cap(filename);
        if(!cap.isOpened())
        {
            std::cerr << "Error. Cant open video";
            return -1;
        }

        int frame_count = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        double fps = cap.get(cv::CAP_PROP_FPS);
        while (true)
        {
            cv::Mat frame;
            bool ret = cap.read(frame);
            if(!ret)
            {
                break;
            }    
            frame = cubedetectcv::detectAndDrawCube(frame.clone());
            cv::imshow("Video", frame);
            char key = cv::waitKey(1);
            if(key =='q') break;
        }
        cap.release();
        cv::destroyAllWindows();
    }
    else{
        //Image
        cv::Mat image = cv::imread(filename);
        if(image.empty())
        {
            std::cerr << "Cant read file \n";
            return -1;
        }

        cv::Mat cubeImage;
        try
        {
            cubeImage = cubedetectcv::detectAndDrawCube(image);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error proccessing image" << '\n';
            return -1;
        }
        
        

        cv::namedWindow("Cube", cv::WINDOW_NORMAL);
        cv::imshow("Cube", cubeImage);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

   
    
    
    
    





    return 0;
}

