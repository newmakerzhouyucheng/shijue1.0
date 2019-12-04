#ifndef ARMOR_DETECT_H
#define ARMOR_DETECT_H
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include "usb_serial/serial_usb.h"
using namespace std;



/****************************************
* @class Name  ArmorPosture
* @brief   检测到装甲板的姿态
*
*****************************************/
class ArmorPosture
{
public:
    ArmorPosture(cv::RotatedRect rect)
    {
        armor_yaw = 0.0;
        armor_pitch = 0.0;
        diatance = 0.0;
        bullet_speed = 0.0;
        target_rect = rect;
        roi_offset_x = 0;
        roi_offset_y = 0;
        isTrue = true;
    }
    double armor_yaw;
    double armor_pitch;
    double diatance;
    double bullet_speed;
    uint armor_type;
    float roi_offset_x;
    float roi_offset_y;
    int armor_color;
    cv::RotatedRect target_rect;//(cv::Point2f(0,0),cv::Point2f(1280,0),cv::Point2f(1280,1024));
    bool isTrue;
private:
};
/****************************************
* @class name  ArmorProcess
* @brief    装甲处理，
*
*****************************************/
class ArmorProcess
{
public:
    bool track_flag;
    bool tml = true;
    cv::Rect GetRoi(cv::Mat &src_image);
    void ArmorDetect(cv::Mat frame,ArmorPosture &fight_info,UsbSerial serial_usb);
    void GetTargetRoi(cv::Mat &frame,ArmorPosture &fight_info);
private:
    cv::Rect last_target;
    void DetectLightBarBgr(cv::Mat &frame,vector<cv::RotatedRect>&light_rect,ArmorPosture fight_info);
    void DetectLightBarHsv(cv::Mat &frame,vector<cv::RotatedRect>&light_rect);
    void DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
    void CheckLightBarRect(vector<cv::RotatedRect>&light_rect,cv::Mat frame);
    void AdjustRotatedRect(cv::RotatedRect &rect);
    void ArmorMatchedRect(std::vector<cv::RotatedRect> &light_rect,std::vector<cv::RotatedRect> &matched_armor);
    void ArmorAngleCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor);
    void ArmorHeightCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor);
    void ArmorDistanceCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor,ArmorPosture fight_info);
    void ArmorRectCheck(std::vector<cv::RotatedRect> &matched_armor,std::vector<cv::RotatedRect> &final_armor,cv::Mat frame);
    void DrawArmorRect(const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
    cv::RotatedRect BoundingArmorRect(cv::RotatedRect left_rect,cv::RotatedRect right_rect);
    cv::Rect2d Rotated2Rect(cv::RotatedRect &rotated_rect);
};

#endif // ARMAR_DETECT_H
