#include "angle_solver.h"


/****************************************
* @funcName void PnPSolver();
* @brief    计算角度，距离
* @para     无
* @return   无
*****************************************/
void AngleSolver::PnPSolver(cv::RotatedRect object_rect,ArmorPosture &fight_info)
{
    Mat rot_vector,translation_vector;
    vector<Point2f> object2d_point;
    GetImage2dPoint(object_rect,object2d_point,fight_info);  //像素坐标系排序
    GetAngleDistance(object2d_point,rot_vector,translation_vector);  //pnp结算

    double tx = translation_vector.at<double>(0,0);
    double ty = translation_vector.at<double>(1,0);
    double tz = translation_vector.at<double>(2,0);
    double dis = sqrt(tx*tx+ty*ty+ tz*tz);

    double angle_yaw = atan2(abs(tx),tz) * 180 / 3.1415926;

    double angle_pitch = atan((abs(ty))/tz * 1.0) * 180 / 3.1415926;
    fight_info.diatance = dis;
    fight_info.armor_yaw = angle_yaw;
    fight_info.armor_pitch = angle_pitch;
    if(tx >0)
    {
        fight_info.armor_yaw = angle_yaw;
    }
    else
    {
         fight_info.armor_yaw = -angle_yaw;
    }
    if(ty > 0)
    {
        fight_info.armor_pitch = angle_pitch;
    }
    else
    {
        fight_info.armor_pitch = -angle_pitch;
    }
    if(dis>= 200 && dis<= 350)
    {
        fight_info.armor_pitch += 1.5;
    }
    else if(dis > 350 && dis<= 450)
    {
        fight_info.armor_pitch += 0.5;
    }
    //cout<<"距离值="<<dis<<"cm"<<"yaw"<<fight_info.armor_yaw<<"Pitch"<<fight_info.armor_pitch<<endl;
}

/****************************************
* @funcName  GetImage2dPoint
* @brief    得到2维像素点
* @para     目标矩形，点向量
* @return   无
*****************************************/
void AngleSolver::GetImage2dPoint(cv::RotatedRect object_rect,std::vector<Point2f> &object2d_point,ArmorPosture fight_info)
{
    cv::Point2f vertices[4];
    object_rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y) {
        lu = vertices[0];
        ld = vertices[1];
    } else {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
        ru = vertices[2];
        rd = vertices[3];
    } else {
        ru = vertices[3];
        rd = vertices[2];
    }
    lu.x += fight_info.roi_offset_x;
    ru.x += fight_info.roi_offset_x;
    ld.x += fight_info.roi_offset_x;
    rd.x += fight_info.roi_offset_x;

    lu.y += fight_info.roi_offset_y;
    ru.y += fight_info.roi_offset_y;
    ld.y += fight_info.roi_offset_y;
    rd.y += fight_info.roi_offset_y;

    object2d_point.clear();
    object2d_point.push_back(lu);
    object2d_point.push_back(ru);
    object2d_point.push_back(rd);
    object2d_point.push_back(ld);
}
/****************************************
* @funcName GetAngleDistance()
* @brief
* @para
* @return
*****************************************/
void AngleSolver::GetAngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Point3f> point3d;
    //小装甲
    //float half_x = 13.8f/2.0f;  //width_target / 2.0;
    float half_x = 13.5f/2.0f;
    float half_y = 5.4f/2.0f;  //height_target / 2.0;
    //三维点
    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    Mat cam_matrix = (Mat_<double>(3,3)<<2132.39692,0,767.66209,0,2166.36857,525.69460,0,0,1);
    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.08189,5.37811,0.03338,0.01634,0);
//    Mat cam_matrix = (Mat_<double>(3,3)<<622.10421,0,355.07792,0,629.61051,256.2444,0,0,1);
//    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.42574,0.29455,-0.00056,0.00119,0);
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);  //旋转向量和旋转矩阵的变换
}
void AngleSolver::GetAngleDistance_big(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Point3f> point3d;
    //小装甲
    float half_x = 21.8f/2.0f;  //width_target / 2.0;
    float half_y = 5.4f/2.0f;  //height_target / 2.0;
    //三维点
    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    Mat cam_matrix = (Mat_<double>(3,3)<<2132.39692,0,767.66209,0,2166.36857,525.69460,0,0,1);
    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.08189,5.37811,0.03338,0.01634,0);
//    Mat cam_matrix = (Mat_<double>(3,3)<<622.10421,0,355.07792,0,629.61051,256.2444,0,0,1);
//    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.42574,0.29455,-0.00056,0.00119,0);
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);  //旋转矩形和旋转矩阵的变换
}
/****************************************
* @funcName GetAngleDistance()
* @brief
* @para
* @return
*****************************************/
void AngleSolver::SolverDataProcess(uint8_t *armor_data,ArmorPosture fight_info)
{
    if(armor_data[7] == 1)
    {
        armor_data[0] = 'F';
        armor_data[1] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.armor_yaw*100) >> 8);
        armor_data[2] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.armor_yaw*100));
        armor_data[3] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.armor_pitch*100) >> 8);
        armor_data[4] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.armor_pitch*100));
        armor_data[5] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.diatance*100) >> 8);
        armor_data[6] = static_cast<uint8_t>(static_cast<int16_t>(fight_info.diatance*100));
        armor_data[7] = 1;
        armor_data[8] = armor_data[0]+armor_data[1]+armor_data[2]+armor_data[3]+armor_data[4]+armor_data[5]+armor_data[6]+armor_data[7]+'E';
        armor_data[9] = 'E';
    }
    else
    {
        armor_data[0] = 'F';
        armor_data[1] = 0;
        armor_data[2] = 2;
        armor_data[3] = 3;
        armor_data[4] = 4;
        armor_data[5] = 0;
        armor_data[6] = 0;
        armor_data[7] = 0;
        armor_data[8] = 'F'+'E';
        armor_data[9] = 'E';
    }
}
/****************************************
* @funcName AngleChange()
* @brief
* @para
* @return
*****************************************/
void AngleChange(ArmorPosture &fight_info)
{
    //fight_info.diatance * 90 - fight_info.armor_pitch;
}



