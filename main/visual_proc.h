#ifndef VISUAL_PROC_H
#define VISUAL_PROC_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>

enum EnemyType
{
    NO_GET = 0,
    BLUE,
    RED
};


class VisualProc
{
public:
    VisualProc()
    {
        mode = 0;
        ArmorType = BLUE;
    }
    void produce();
    void consumer();
    void WriteVideo();
    void Serial();
    int mode;
    EnemyType ArmorType;
    uint enemy_color = 0;
private:
    std::mutex mtx;
    std::vector<cv::Mat> vec_buff;
    std::condition_variable cv_thread;
};

#endif // VISUAL_PROC_H
