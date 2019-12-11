#include "open_camera.h"

bool OpenCamera::OpenFrame(dvpStr camera_congig_file)
{
   status = dvpRefresh(&n);
   if (status == DVP_STATUS_OK)
   {
       // 枚举最多16台相机的信息
       if (n > 16)
       {
           n = 16;
       }
       cout <<"有"<<n<<"台相机"<< endl;
       for (i = 0; i < n; i++)
       {
           // 逐个枚举出每个相机的信息
           status = dvpEnum(i, &info[i]);
           if (status != DVP_STATUS_OK)
           {
              cout << "枚举NO"  << endl;
           }
           else
           {
              cout <<"枚举OK"<< endl;
           }
       }
   }
   status = dvpOpenByName(info[0].FriendlyName,OPEN_NORMAL,&m_handle);
   if (status == DVP_STATUS_OK)
   {
       cout<<"ok!!!"<<endl;//// QMessageBox::about(NULL,"About","Open the camera fail!");
   }
   status = dvpGetStreamState(m_handle,&state);
   if (status != DVP_STATUS_OK)
   {
       cout<<"Get the stream state fail!"<<endl; //QMessageBox::about(NULL,"About","Get the stream state fail!");
   }
   if (state == STATE_STOPED)
   {
       status = dvpGetTriggerState(m_handle,&bTrigStatus);
       if (status != DVP_STATUS_FUNCTION_INVALID)
       {
           // 在启动视频流之前先设置为触发模式
           status = dvpSetTriggerState(m_handle,SoftTriggerFlag ? true : false);
           if (status != DVP_STATUS_OK)
           {
                //QMessageBox::about(NULL,"About","Set status of trigger fail!");
           }
           else
           {
               cout<<"Set status of trigger"<<endl;
           }
       }
       else
       {
           cout<<"sss"<<endl;//ui->groupBox_trigger->setEnabled(false);
       }

       status = dvpStart(m_handle);
       if (status != DVP_STATUS_OK)
       {
           //QMessageBox::about(NULL,"About","Start the video stream fail!");
       }
   }
   dvpStr path= camera_congig_file;
   //dvpStr path1= "/home/newmaker/rm_task/Test/test/1280_1024.ini";
   dvpLoadConfig(m_handle,path);
   return true;
}

cv::Mat OpenCamera::GetFrame()
{
    cv::Mat mat_src;
    status = dvpGetFrame(m_handle, &m_pFrame, &pBuffer,30000);
    mat_src = cv::Mat(m_pFrame.iHeight, m_pFrame.iWidth, CV_8UC3, pBuffer);
    return mat_src;
}




//cv::Mat QImageToMat(QImage image)
//{
//    cv::Mat mat;
//    switch (image.format())
//    {
//    case QImage::Format_ARGB32:
//    case QImage::Format_RGB32:
//    case QImage::Format_ARGB32_Premultiplied:
//        mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
//        break;
//    case QImage::Format_RGB888:
//        mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
//        cv::cvtColor(mat, mat, CV_BGR2RGB);
//        break;
//    case QImage::Format_Indexed8:
//        mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
//        break;
//    }
//    return mat;
//}
