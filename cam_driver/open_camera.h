#ifndef OPEN_CAMERA_H
#define OPEN_CAMERA_H

#include<opencv2/opencv.hpp>
#include "DVPCamera.h"
#include <iostream>
using namespace std;

class OpenCamera
{
public:
  bool OpenFrame(dvpStr camera_congig_file);
  ~OpenCamera()
  {
     dvpClose(m_handle);
  }
  cv::Mat GetFrame();
  dvpFrame  m_pFrame;                   // 采集到的图像的结构体指针
  void *    pBuffer;                    // 采集到的图像的内存首地址
  dvpStatus status;
  //dvpStatus status1;
  dvpUint32 i = 0;
  dvpUint32 n = 0;
  dvpCameraInfo info[16];
  dvpHandle  m_handle;
  dvpStreamState state;
  dvpDoubleDescr ExpDescr;
  dvpFloatDescr sAnalogGainDescr;
  bool bTrigStatus;
  bool SoftTriggerFlag;

};
#endif // OPEN_CAMERA_H
