
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
        std::cout << "Enter file path : ";
        std::cin >> filename;
    }
    #ifdef DEBUG
    std::cout << "Defining the file type : "<< filename << std::endl;
    #endif

    //Check file type 
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
    return 0;
}

void showVideoCube(std::string filename)
{
     #ifdef DEBUG
    std::cout << "\n";
    #endif
        cv::VideoCapture cap(filename);
        if(!cap.isOpened())
        {
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

void showImageCube(std::string filename)
{
 cv::Mat image = cv::imread(filename);
        if(image.empty())
        {
        }

        cv::Mat cubeImage;
        try
        {
            cubeImage = cubedetectcv::detectAndDrawCube(image);
        }
        catch(const std::exception& e)
        {
        }
        
        

        cv::namedWindow("Cube", cv::WINDOW_NORMAL);
        cv::imshow("Cube", cubeImage);

        cv::waitKey(0);
        cv::destroyAllWindows();
}
