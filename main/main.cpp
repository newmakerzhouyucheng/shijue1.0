#include <iostream>
#include <thread>
#include "visual_proc.h"
#include <opencv2/opencv.hpp>
#include "cam_driver/open_camera.h"


using namespace cv;
using namespace std;

int main()
{
    VisualProc visual_proc;
    std::thread ImgProduce(&VisualProc::produce,&visual_proc);  
    //std::thread ImgConsume(&VisualProc::consumer,&visual_proc);
    ImgProduce.join();
    //ImgConsume.join();
    return 0;
}
