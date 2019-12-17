#include "armor_detect/armor_detect.h"
#include "Info_solver/angle_solver.h"
#include "cam_driver/open_camera.h"
#include "usb_serial/serial_usb.h"
#include "main/visual_proc.h"


#define DEBUG_SHOW
#define DEBUG_DLB_SHOW


#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
cv::Rect ArmorProcess::GetRoi(cv::Mat &src_image)
{
    Size img_size = src_image.size();
    Rect rect_roi;

    if(tml)
    {
    tml = false;
    last_target = Rect(0,0,img_size.width,img_size.height);
    }

    if(last_target.width == img_size.width)
    {
        rect_roi = last_target;
        return rect_roi;
    }
    else
    {
            Rect rect_tmp = last_target;
            int x = rect_tmp.x;
            int y = rect_tmp.y;
            int w = rect_tmp.width;
            int h = rect_tmp.height;
            rect_roi = Rect(x,y,w,h);
            return rect_roi;
    }

}

void ArmorProcess::ArmorDetect(cv::Mat frame,ArmorPosture &fight_info,UsbSerial serial_usb)
{
    int distance_mode;
    AngleSolver armor_Solver;
    vector<cv::RotatedRect>frame_dis;    
    vector<cv::RotatedRect>frame_sec;
    vector<cv::RotatedRect>frame_ang;
    vector<cv::RotatedRect>frame_final;
    cv::RotatedRect frame_std1;
    cv::RotatedRect frame_std2;
    cv::RotatedRect frame_std3;
    uint8_t pc_data[10];

    /*DetectLightBar   Way of RGB  */
    DetectLightBarBgr(frame,frame_dis,fight_info);
    /*DetectLightBar   Way of HSV  */
    //DetectLightBarHsv(src_image,light_rect);

    CheckLightBarRect(frame_dis,frame,distance_mode);  //角度筛选
    
    ArmorMatchedRect(frame_dis,frame_sec);  //灯条匹配
    if(frame_sec.size()>0)
    {
        ArmorRectCheck(frame_dis,frame_sec,frame_ang,distance_mode,frame);  //大框长宽比角度筛选
        if(frame_ang.size()>0)
        {
            //ArmorAngleCheck(frame_ang,frame_std1);  //大框最小角度框筛选1
            ArmorHeightCheck(frame_ang,frame_std2);  //大框面积筛选
            //ArmorDistanceCheck(frame_ang,frame_std3,fight_info);  //大框最近距离筛选3
            //if(frame_std1.size == frame_std2.size)
            {
                //frame_final.push_back(frame_std1);
            }
            //else if(frame_std1.size == frame_std3.size)
            {
                //frame_final.push_back(frame_std3);
            }
            //else if(frame_std2.size == frame_std3.size)
            {
                frame_final.push_back(frame_std2);
            }
        }else
        {
            frame_final = frame_ang;
        }

        //if(!frame_final.empty())
        //{
            //point_s.clear();
            //frame_final[0].points(point_roi);
            //for(int i =0; i < 4; i++)
            //{
                //point_s.push_back(point_roi[i]);
            //}
            //last_target = cv::boundingRect(point_s);
            for(uint i = 0; i< frame_final.size(); i++)
            {
                pc_data[7] = 1;
                DrawArmorRect(frame,frame_final[i],cv::Scalar(160, 15, 190),2);  //画大框
                armor_Solver.PnPSolver(frame_final[i],fight_info);  //包含pnp结算和数据处理
                armor_Solver.SolverDataProcess(pc_data,fight_info);
            }
    }
    else
    {
        //TO do
    }
    if(frame_final.size() < 1)
    {
        pc_data[0] = 'F';
        pc_data[1] = 0;
        pc_data[2] = 0;
        pc_data[3] = 0;
        pc_data[4] = 0;
        pc_data[5] = 0;
        pc_data[6] = 0;
        pc_data[7] = 0;
        pc_data[8] = 'F'+'E'+pc_data[1]+pc_data[2]+pc_data[3]+pc_data[4]+pc_data[5]+pc_data[6]+pc_data[7];
        pc_data[9] = 'E';
        cv::RotatedRect rect_temp(cv::Point2f(0,0),cv::Point2f(1280,0),cv::Point2f(1280,1024));
        fight_info.target_rect = rect_temp;
        fight_info.isTrue = false;
    }
    else
    {
        fight_info.target_rect = frame_final[0];
        fight_info.isTrue = true;
    }
    /*************************** DEBUG SHOW ******************************************/
    serial_usb.SerialSendData(pc_data);
}

void ArmorProcess::ArmorAngleCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor)
{
    //大框角度转化
    for(uint8_t i = 0; i < matched_armor.size(); i++)
    {
        if(matched_armor[i].angle < -90)
        {
            matched_armor[i].angle += 180;
        }
        else
        {
            matched_armor[i].angle = abs(matched_armor[i].angle);
        }
    }
    //选出最小角度
    sort(matched_armor.begin(),matched_armor.end(),[](const RotatedRect& ld1, const RotatedRect& ld2)
    {
        return  ld1.angle < ld2.angle;
    });
    final_armor = matched_armor[0];
}

void ArmorProcess::ArmorHeightCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor)
{
    sort(matched_armor.begin(),matched_armor.end(),[](const RotatedRect& ld1, const RotatedRect& ld2)
    {
        return ld1.size.area() > ld2.size.area();
    });
    double area_minus = static_cast<double>(matched_armor[0].size.area() - matched_armor[1].size.area());
    double a = static_cast<double>(MAX(matched_armor[0].size.area(), matched_armor[1].size.area()));
    double b = static_cast<double>(a/area_minus);
    //if(b > 2.2)
    //{
        //if(matched_armor[0].center.x < matched_armor[1].center.x)
        //{
            //final_armor = matched_armor[0];
        //}else
        //{
            //final_armor = matched_armor[1];
        //}
    //}
    if(b > 2.2)
    {
        if(abs(matched_armor[0].center.x - 640) < abs(matched_armor[1].center.y - 640))
        {
            final_armor = matched_armor[0];
        }else
        {
            final_armor = matched_armor[1];
        }
    }
    final_armor = matched_armor[0];
}

void ArmorProcess::ArmorDistanceCheck(std::vector<cv::RotatedRect> &matched_armor,cv::RotatedRect &final_armor,ArmorPosture fight_info)
{
    AngleSolver a;
    cv::Mat rot_vector,translation_vector;
    vector<Point2f> object2d_point;
    vector<double> distance;
    for(uint8_t i = 0; i < matched_armor.size(); i++)
    {
    a.GetImage2dPoint(matched_armor[i],object2d_point,fight_info);  //像素坐标系排序
    a.GetAngleDistance(object2d_point,rot_vector,translation_vector);  //pnp结算
    double tx = translation_vector.at<double>(0,0);
    double ty = translation_vector.at<double>(1,0);
    double tz = translation_vector.at<double>(2,0);
    double dis = sqrt(tx*tx+ty*ty+ tz*tz);
    distance.push_back(dis);
    }
    //sort(distance.begin(),distance.end(),[](const double& a, const double& b)
    //{
        //return a > b;
    //});
    std::vector<double>::iterator smallist = std::min_element(std::begin(distance),std::end(distance));
    uint8_t d = static_cast<uint8_t>(std::distance(std::begin(distance),smallist));
    final_armor = matched_armor[d];
}

/****************************************
* @funcName  ArmorRectCheck
* @brief    装甲筛选
* @para     无
* @return   无
*****************************************/
void ArmorProcess::ArmorRectCheck(std::vector<cv::RotatedRect>&light_rect,std::vector<cv::RotatedRect> &matched_armor,std::vector<cv::RotatedRect> &final_armor,int &distance_mode,cv::Mat &frame)
{
    final_armor.clear();
    sort(light_rect.begin(),light_rect.end(),[](const RotatedRect& ld1, const RotatedRect& ld2)
    {
        return ld1.size.area() > ld2.size.area();
    });
    float area = (light_rect[0].size.area() + light_rect[1].size.area())/2;
    std::cout << "area=" << area << std::endl;

    for(uint8_t i = 0; i< matched_armor.size();i++)
    {
        auto rect_h = min(matched_armor[i].size.height,matched_armor[i].size.width);
        auto rect_w = max(matched_armor[i].size.height,matched_armor[i].size.width);
        auto ratio_h_w = rect_w * 1.0f / rect_h;

           //测试大框长宽比范围
           char string6[10];
           double h_w_string = static_cast<double>(ratio_h_w);
           sprintf(string6,"%.1f",h_w_string);
           cv::putText(frame,string6,matched_armor[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
           //std::cout << ratio_h_w << std::endl;

           //测试大框角度范围
           //char string7[10];
           //double a_string = static_cast<double>(matched_armor[i].angle);
           //sprintf(string7,"%.1f",a_string);
           //cv::putText(frame,string7,matched_armor[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
           //std::cout << matched_armor[i].angle << std::endl;

           //测试大框面积范围
           //char string8[10];
           //double a_string = static_cast<double>(matched_armor[i].size.area());
           //sprintf(string8,"%.1f",a_string);
           //cv::putText(frame,string8,matched_armor[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
           //std::cout << matched_armor[i].size.area() << std::endl;

        if(abs(abs(matched_armor[i].angle) - 90) > 75)
        {
            if(distance_mode == 1)
            {
                if(2463 < area && area < 3300)
                {
                    if(ratio_h_w < 2.45f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.75f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }
            }else if(distance_mode == 2)
            {
                if(3284 < area && area < 1848)
                {
                    if(ratio_h_w < 2.45f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.75f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 3)
            {
                if(450 < area && area < 1080)
                {
                    if(ratio_h_w < 2.45f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.75f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 4)
            {
                if(300 < area && area < 600)
                {
                    if(ratio_h_w < 2.35f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.65f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 5)
            {
                if(213 < area && area < 415)
                {
                    if(ratio_h_w < 2.35f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.65f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 6)
            {
                if(189 < area && area < 284)
                {
                    if(ratio_h_w < 2.25f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.55f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 7)
            {
                if(145 < area && area < 252)
                {
                    if(ratio_h_w < 2.25f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.55f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 8)
            {
                if(123 < area && area < 193)
                {
                    if(ratio_h_w < 2.15f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.45f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }else if(distance_mode == 9)
            {
                if(area < 163)
                {
                    if(ratio_h_w < 2.05f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }
                }else
                {
                    if(ratio_h_w < 2.35f)
                    {
                        final_armor.push_back(matched_armor[i]);
                    }

                }

            }

        }
    }
}
/****************************************
* @funcName DetectLightBarColor
* @brief    检测装甲灯条  使用颜色空间BGR
* @para     Mat 原始图像 ,
* @return   RotatedRect 灯条旋转矩形向量
*****************************************/
void ArmorProcess::
DetectLightBarBgr(cv::Mat &frame,vector<cv::RotatedRect>&light_rect,ArmorPosture fight_info)
{
    light_rect.clear();
    cv::Mat gray;
    cv::Mat color_minus;
    cv::Mat gray_binary;
    cv::Mat color_minus_binary;
    cv::Mat combine_binary;
    cv::Mat light_rectangle = cv::Mat::zeros(frame.size(), CV_8UC3);

    std::vector<cv::Mat> channels;
    std::vector<cv::RotatedRect> light_line_conbine;
    std::vector<std::vector<cv::Point>> gray_binary_contours;
    std::vector<std::vector<cv::Point>> combine_binary_contours;

    cv::cvtColor(frame,gray,CV_BGR2GRAY);
    cv::blur(gray,gray,cv::Size(3,3));
    cv::split(frame,channels);
    if(fight_info.armor_color == BLUE)
    {
        subtract(channels[0],channels[1],color_minus);
        cv::threshold(gray,gray_binary,100,255,CV_THRESH_BINARY);
        cv::threshold(color_minus,color_minus_binary,60,255,CV_THRESH_BINARY);
    }
    if(fight_info.armor_color == RED)
    {
        subtract(channels[2],channels[1],color_minus);
        cv::threshold(gray,gray_binary,100,255,CV_THRESH_BINARY);
        cv::threshold(color_minus,color_minus_binary,60,255,CV_THRESH_BINARY);
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(color_minus_binary,color_minus_binary,element,cv::Point(-1,-1),1);

    combine_binary = color_minus_binary & gray_binary;
    cv::dilate(combine_binary,combine_binary,element,cv::Point(-1,-1),2);

    findContours(gray_binary,gray_binary_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    findContours(combine_binary,combine_binary_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    light_line_conbine.reserve(combine_binary_contours.size());

    //使用两个不同的二值化阈值同时进行灯条提取,减少环境光照对二值化这个操作的影响
    for(uint i = 0; i < combine_binary_contours.size(); ++i)
    {
        //for(uint j = 0; j < gray_binary_contours.size(); ++j)
        {
            //if(cv::pointPolygonTest(gray_binary_contours[j],combine_binary_contours[i][0],true) >= 0.0)
            {
                cv::RotatedRect light_line = cv::minAreaRect(combine_binary_contours[i]);
                light_line_conbine.push_back(light_line);
                DrawRotatedRect(light_rectangle, light_line,cv::Scalar(255,0,255), 1);  //画框
            }
        }
        //测底盘装甲板灯条小矩形角度范围
        //char string1[10];
        //double angle_string = static_cast<double>(light_line.angle);
        //sprintf(string1, "%.1f", angle_string);
        //cv::putText(light_rectangle,string1,light_line.center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
        //std::cout << light_line.angle << std::endl;

        //测底盘装甲板灯条小矩形面积范围
        //char string2[10];
        //double area_string = static_cast<double>(light_line.size.area());
        //sprintf(string2,"%.1f",area_string);
        //cv::putText(light_rectangle,string2,light_line.center,cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,255,0),2);
        //std::cout << light_line.size.area() << std::endl;

    }

    //imshow("gray",gray);
    //imshow("color_minus",color_minus);
    //imshow("gray_binary",gray_binary);
    //imshow("color_minus_binary",color_minus_binary);
    //imshow("combine_binary",combine_binary);
    imshow("light_rectangle",light_rectangle);

    light_rect = light_line_conbine;
 
}
/****************************************
* @funcName ArmorMatchedRect
* @brief    从灯条中匹配装甲
* @para     vector<RotatedRect> &light_rect
* @return   无
*****************************************/
void ArmorProcess::ArmorMatchedRect(std::vector<cv::RotatedRect> &light_rect,std::vector<cv::RotatedRect> &matched_armor)
{
    for(uint8_t d = 0; d < light_rect.size(); d++)
    {
        if(light_rect[d].angle >= 0)
        {
            light_rect[d].angle -= 90;
        }
    }
    matched_armor.clear();
    for(uint8_t i = 0;i < light_rect.size();i++)
    {
        for(uint8_t j = i+1;j < light_rect.size();j++)
        {
            auto sub_center_x = abs(light_rect[i].center.x - light_rect[j].center.x);      //中心点x坐标差值
            auto sub_center_y = abs(light_rect[i].center.y - light_rect[j].center.y);      //中心点y坐标差值
            float sub_height = abs(light_rect[i].size.height - light_rect[j].size.height); //高度差值
            float area_ratio = 0;                                                          //两个灯条的面积比例
            if(light_rect[i].size.area() < light_rect[j].size.area())
            {
                area_ratio = (light_rect[j].size.area())*1.0f/light_rect[i].size.area();
            }
            else
            {
                area_ratio = (light_rect[i].size.area())*1.0f/light_rect[j].size.area();
            }
            if((20 < sub_center_x && sub_center_x < 1280) && (0 <= sub_center_y && sub_center_y < 30) && area_ratio <= 2.2f && (0 <= sub_height && sub_height < 30))
            {
                cv::RotatedRect  armor_rect;
                armor_rect = BoundingArmorRect(light_rect[i],light_rect[j]);  //大框
                matched_armor.push_back(armor_rect);
            }
        }
    }
}
/****************************************
* @funcName BoundingArmorRect
* @brief    找到包围左右灯条的最小矩形
* @para
* @return   包围左右灯条的最小矩形
************************************/
cv::RotatedRect ArmorProcess::BoundingArmorRect(cv::RotatedRect left_rect,cv::RotatedRect right_rect)
{
    const cv::Point & pl = left_rect.center, & pr = right_rect.center;
    cv::Point2f center;
    center.x = (pl.x + pr.x) / 2.0f;
    center.y = (pl.y + pr.y) / 2.0f;
    cv::Size2f wh_l = left_rect.size;
    cv::Size2f wh_r = right_rect.size;
    float width =static_cast<float>(POINT_DIST(pl, pr));
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right_rect.center.y - left_rect.center.y, right_rect.center.x - left_rect.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), (angle * 180.0f)/static_cast<float>(CV_PI));
}
/****************************************
* @funcName AdjustRotatedRect
* @brief    调整矩形的width和heigth
* @para     旋转矩形
* @return   无
*****************************************/
void ArmorProcess::AdjustRotatedRect(cv::RotatedRect &rect)  
{

    if(rect.size.width >= rect.size.height)
    {
        auto temp = rect.size.height;
        rect.size.height = rect.size.width;
        rect.size.width = temp;
        rect.angle += 90;
    }
    int a = static_cast<int>(rect.angle);
    if(a == 90)
    {
        rect.angle = 0;
    }
}
/****************************************
* @funcName CheckLightBarRect
* @brief    排除干扰的灯条矩形
* @para     vector<RotatedRect>&light_rect
* @return   无
*****************************************/
void ArmorProcess::CheckLightBarRect(vector<cv::RotatedRect>&light_rect,cv::Mat frame, int &distance_mode)
{
    std::vector<cv::RotatedRect> temp_light_rect;
    cv::Mat frame_select = cv::Mat::zeros(frame.size(),CV_8UC3);
    temp_light_rect.clear();
    for(uchar i = 0; i < light_rect.size(); i++)
    {
      AdjustRotatedRect(light_rect[i]);  //长宽角度变换
    }
    for(const auto &light :light_rect)
    {
        if(abs(light.angle) >= 0 && abs(light.angle) <= 20 && light.size.area()> 60 )
        {
            temp_light_rect.push_back(light);
            DrawRotatedRect(frame_select ,light,cv::Scalar(0,0,255),1);
        }
    }

    for(uchar i = 0; i < temp_light_rect.size(); i ++)
    {
        //测试底盘应该匹配的小灯条中心X差值
        //char string3[10];
        //double x_string = static_cast<double>(temp_light_rect[i].center.x);
        //sprintf(string3,"%.1f",x_string);
        //cv::putText(frame_select,string3,temp_light_rect[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
        //std::cout << temp_light_rect[i].center.x << std::endl;

        //测试底盘应该匹配的小灯条中心y差值
        //char string5[10];
        //double x_string = static_cast<double>(temp_light_rect[i].center.y);
        //sprintf(string5,"%.1f",x_string);
        //cv::putText(frame_select,string5,temp_light_rect[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
        //std::cout << temp_light_rect[i].center.y << std::endl;

        //测试底盘应该匹配的小灯条高度差值
        char string4[10];
        double h_string = static_cast<double>(temp_light_rect[i].size.height);
        sprintf(string4,"%.1f",h_string);
        cv::putText(frame_select,string4,temp_light_rect[i].center,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),2);
        //std::cout << temp_light_rect[i].size.height << std::endl;
    }

    if(temp_light_rect.size() > 0)
    {
        sort(temp_light_rect.begin(),temp_light_rect.end(),[](const RotatedRect& ld1, const RotatedRect& ld2)
        {
            return ld1.size.height > ld2.size.height;
        });

        if(120 <= temp_light_rect[0].size.height)
        {
            distance_mode = 1;
        }else if(60 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 120)
        {
            distance_mode = 2;
        }else if(41 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 60)
        {
            distance_mode = 3;
        }else if(32 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 41)
        {
            distance_mode = 4;
        }else if(26 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 32)
        {
            distance_mode = 5;
        }else if(22 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 26)
        {
            distance_mode = 6;
        }else if(20 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 22)
        {
            distance_mode = 7;
        }else if(18 <= temp_light_rect[0].size.height && temp_light_rect[0].size.height < 20)
        {
            distance_mode = 8;
        }else if(temp_light_rect[0].size.height < 18)
        {
            distance_mode = 9;
        }

    }
    imshow("frame_select",frame_select);
    light_rect = temp_light_rect;
}

/****************************************
* @funcName DrawRotatedRect
* @brief    画旋转矩形
* @para     Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness
* @return   无
*****************************************/
void ArmorProcess::DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];
    rect.points(vertex);
    for(int i = 0; i < 4; i++) {
        cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
    }
}
/****************************************
* @funcName
* @brief
* @para     无
* @return   无
*****************************************/
cv::Rect2d ArmorProcess::Rotated2Rect(cv::RotatedRect &rotated_rect)
{
    cv::Point2f pts[4];
    rotated_rect.points(pts);
    auto min_y = static_cast<double>(min(min(pts[0].y,pts[1].y),min(pts[2].y,pts[3].y)));
    auto max_y = static_cast<double>(max(max(pts[0].y,pts[1].y),max(pts[2].y,pts[3].y)));
    auto min_x = static_cast<double>(min(min(pts[0].x,pts[1].x),min(pts[2].x,pts[3].x)));
    auto max_x = static_cast<double>(max(max(pts[0].x,pts[1].x),max(pts[2].x,pts[3].x)));
    cv::Rect2d temp_rect(min_x,min_y,max_x,max_y);
    return temp_rect;
}
/****************************************
* @funcName DrawArmorRect
* @brief    在给定的图像上画出矩形框
* @para     const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness
* @return   无
*****************************************/
void ArmorProcess::DrawArmorRect(const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];
    rect.points(vertex);
    for (int i=0; i<4; i++)
    cv::line(image, vertex[i], vertex[(i+1)%4], color, thickness);
}

void ArmorProcess::GetTargetRoi(cv::Mat &frame,ArmorPosture &fight_info)
{
    cv::RotatedRect temp_rect = fight_info.target_rect;
    cv::Mat Roi_frame;
    cv::Point2f vertices[4];
    cv::Point2f lu, ld, ru, rd;
    temp_rect.points(vertices);
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    int left_limit_x = static_cast<int>(vertices[0].x);
    int right_limit_x = static_cast<int>(vertices[3].x);
    //cout<<left_limit_x<<"-"<<right_limit_x<<endl;

    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }
    int up_limit_y = static_cast<int>(lu.y < ru.y ? lu.y:ru.y);
    int down_limit_y =static_cast<int>(ld.y > rd.y ? ld.y:rd.y);
    //cout<<up_limit_y<<"-"<<down_limit_y<<endl;

    cv::Rect roi_rect;
    left_limit_x = left_limit_x - 200;
    if(left_limit_x <= 0)
    {
        left_limit_x = 0;
    }
  //  cout<<"POINT"<<left_limit_x<<endl;
    up_limit_y = up_limit_y - 200;
    if(up_limit_y <= 0)
    {
        up_limit_y = 0;
    }
  //  cout<<"POINTY"<<up_limit_y<<endl;
    right_limit_x = right_limit_x + 200;
    if(right_limit_x >= 1280)
    {
        right_limit_x = 1280;
    }
    down_limit_y = down_limit_y + 200;
    if(down_limit_y >= 1024)
    {
        down_limit_y = 1024;
    }
    roi_rect.x = left_limit_x;
    roi_rect.y = up_limit_y;
    roi_rect.width = right_limit_x - left_limit_x;
    roi_rect.height = down_limit_y - up_limit_y;
 //   cout<<roi_rect.x<<"--"<<roi_rect.y<<endl;
 //   cout<<right_limit_x - left_limit_x<<"--"<<down_limit_y - up_limit_y<<endl;
    fight_info.roi_offset_x = left_limit_x;
    fight_info.roi_offset_y = up_limit_y;
    Roi_frame = frame(Rect(left_limit_x,up_limit_y,right_limit_x - left_limit_x,down_limit_y - up_limit_y));
    frame = Roi_frame;
}


