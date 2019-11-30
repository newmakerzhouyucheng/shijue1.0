#include "visual_proc.h"
#include "cam_driver/open_camera.h"
#include "armor_detect/armor_detect.h"
#include "usb_serial/serial_usb.h"
#include <thread>
#include <chrono>

#define DEBUG_SHOW

static UsbSerial usb_serial;

/****************************************
* @funcName produce()
* @brief    图像处理线程
* @para     无
* @return   无
*****************************************/
void VisualProc::produce()
{
    cv::VideoCapture cap("/home/newmaker/ZZZ/6m.wmv");
    //dvpStr cam_config_path = "/home/newmaker/ZZZ/test_1.ini";
    //usb_serial.SerialInit();
    //OpenCamera cam_num_1;
    //cam_num_1.OpenFrame(cam_config_path);
    cv::Mat src_image;

    ArmorProcess armor;
    cv::RotatedRect rect(cv::Point2f(0,0),cv::Point2f(1280,0),cv::Point2f(1280,1024));
    ArmorPosture fight_info(rect);

    while (true)
    {
        cap >> src_image;
        //src_image = cam_num_1.GetFrame();
        //cv::resize(src_image,src_image,cv::Size(640,480));
        double time = cv::getTickCount();

        if(ArmorType != NO_GET)
        {
            fight_info.armor_color = ArmorType;
            if(mode == 0)
            {
                //cv::Rect roi = armor.GetRoi(src_image);
                //src_image = src_image(roi);
                armor.ArmorDetect(src_image,fight_info,usb_serial);
            }
            else if (mode == 1)
            {
                //BUFF;
            }
        }
        time = (cv::getTickCount() - time)/cv::getTickFrequency();
#ifdef DEBUG_SHOW
        char string[10];
        sprintf(string, "%.2f", time*1000);      // 帧率保留两位小数
        std::string fpsString("FPS:");
        fpsString += string;                    // 在"FPS:"后加入帧率数值字符串
        std::string fpsString1("ms");
        fpsString +=fpsString1;
        //图像矩阵, string型文字内容 ,文字坐标，以左下角为原点 ,字体类型 ,字体大小,字体颜色,
        cv::putText(src_image, fpsString,cv::Point(5, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        if(ArmorType == NO_GET)
        {
            std::string str = "NO_COLOR";
            cv::putText(src_image, fpsString,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }
        else if(ArmorType ==BLUE)
        {
            std::string str = "Fight Blue";
            cv::putText(src_image, str,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }
        else
        {
            std::string str = "Fight Red";
            cv::putText(src_image, str,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }
        imshow("src_image",src_image);
        if(cv::waitKey(1) == 'q')
        {
            cv::destroyAllWindows();
            break;
        }
#endif
        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}
void VisualProc::consumer()
{
    uint8_t rec_data[3];
    static int num = 0;
    while (true)
    {
        /************************/
        if(num < 10000)
        {
            num++;
            cout<<"This a Bug"<<endl;
        }
        /*************************/
        else
        {
            /*temp*/
            ArmorType = BLUE;
            num = 20000;
            usb_serial.SerialRecData(rec_data);
            if(rec_data[0] == 'F')
            {
                enemy_color = 1;
            }
            if(rec_data[0] == 'q')
            {break;}
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
