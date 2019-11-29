#include "energy_buff.h"


void EnergyBuff::EnergyDetect(EnergyInfo &energy_buff,int &game_mode)
{
    dvpStr path= "/home/newmaker/rm_task/TJK/auto_shoot/cam_file/test_1.ini";
    OpenCamera camera_one;
    camera_one.OpenFrame(path);
    cout << "成功";
    cout << "开始\n";
    //VideoCapture cap(0);
    cv::Mat gray_image;
    cv::Mat rgb_image;
    cv::Mat gray_image_binary;
    cv::Mat rgb_image_binary;
    cv::Mat final_image;
    while (1)
    {
        cv::Mat frame;
        //防止丢帧意外卡死
        frame = camera_one.GetFrame();
        //cap >> frame;
        //resize(frame,frame,cv::Size(640,480));
        cvtColor(frame,gray_image,cv::COLOR_BGR2GRAY);
        vector<cv::Mat> rgb_channel;
        split(frame,rgb_channel);
#ifdef BULE
        subtract(rgb_channel[0],rgb_channel[1],rgb_image);
#else
        subtract(rgb_channel[2],rgb_channel[1],rgb_image);
#endif

        threshold(gray_image,gray_image_binary,240,255,cv::THRESH_BINARY);
    //        dilate(gray_image_binary,gray_image_binary,Mat());
    //        threshold(rgb_image,rgb_image_binary,50,255,THRESH_BINARY);

        final_image = gray_image_binary;// & rgb_image_binary;
        imshow("final_image",final_image);
        floodFill(final_image,cv::Point(1,1),cv::Scalar(255),0,cv::FLOODFILL_FIXED_RANGE);
        imshow("漫水填充",final_image);
        threshold(final_image, final_image, 70, 255, cv::THRESH_BINARY_INV);

        vector<vector<cv::Point>> contours;
        findContours(final_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        vector<cv::RotatedRect> aim_rect;
        for (size_t i = 0; i < contours.size(); i++)
        {
            vector<cv::Point> points;
            double area = contourArea(contours[i]);
            if (area < 50 )//if (area < 50 || 1e4 < area)
                continue;

            drawContours(frame, contours, static_cast<int>(i), cv::Scalar(0,255,0), 3);
            points = contours[i];
            cv::RotatedRect rrect = fitEllipse(points);
            cv::Point2f* vertices = new cv::Point2f[4];
            rrect.points(vertices);
            float aim = rrect.size.height/rrect.size.width;
            cout<<"aim = "<<aim<<endl;
            if(aim > 1.6 && aim < 2.6)
            {
                for (int j = 0; j < 4; j++)
                {
                    cv::line(frame, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255),4);
                }
                float middle = 100000;

                for(size_t j = 1;j < contours.size();j++)
                {
                    vector<cv::Point> pointsA;
                    double area = contourArea(contours[j]);
                    if (area < 50 || 1e4 < area)
                    continue;

                    pointsA = contours[j];

                    cv::RotatedRect rrectA = fitEllipse(pointsA);

                    cv::Point2f* vertices1 = new cv::Point2f[4];
                    rrectA.points(vertices1);

                    float aimA = rrectA.size.height/rrectA.size.width;

                    if(aimA > 3.0)
                    {
                        for (int j = 0; j < 4; j++)
                        {
                            cv::line(frame, vertices1[j], vertices1[(j + 1) % 4], cv::Scalar(255, 0, 255),4);
                        }
                        float distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                        (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));
                        cout<<distance<<endl;

                        if (middle > distance )
                            middle = distance;
                    }
                }
                if( middle >140)  //60
                {                               //这个距离也要根据实际情况调,和图像尺寸和物体远近有关。
                    aim_rect.push_back( rrect);
                    cv::circle(frame,cv::Point(rrect.center.x,rrect.center.y),15,cv::Scalar(100,23,125),2);
                }
            }
        }
        cout<<aim_rect.size()<<endl;
        imshow("BUFF",frame);
        imshow("gray_image",gray_image);
        // imshow("rgb_image",rgb_image);
        imshow("gray_image_binary",gray_image_binary);
        // imshow("rgb_image_binary",rgb_image_binary);
        if(cv::waitKey(1)=='d')
        {
            game_mode = 0;
            camera_one.~OpenCamera();
            cv::destroyAllWindows();
            break;
        }
        if(cv::waitKey(1)=='q')
        {
            game_mode = 3;
            cv::destroyAllWindows();
            break;
        }
    }



}
