#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include "armor_detect/armor_detect.h"
using namespace std;
using namespace cv;


/****************************************
* @className AngleSolver
* @brief     计算目标物体和装甲的相对姿态
* @
* @
*****************************************/
class AngleSolver
{
public:
    void PnPSolver(cv::RotatedRect object_rect,ArmorPosture &fight_info);
    void SolverDataProcess(uint8_t *armor_data,ArmorPosture fight_info);
    void AngleChange(ArmorPosture &fight_info);
private:
    void GetImage2dPoint(cv::RotatedRect object_rect,std::vector<Point2f> &object2d_point,ArmorPosture fight_info);
    void GetAngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
    void GetAngleDistance_big(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
};












#endif // ANGLE_SOLVER_H
