#ifndef ENERGY_BUFF_H
#define ENERGY_BUFF_H
#include<opencv2/opencv.hpp>
#include "open_camera.h"
#include<iostream>
using namespace std;
class EnergyInfo
{
public:
    EnergyInfo(int mode)
    {
        energy_mode = mode;
    }
    float aim_yaw;
    float aim_pitch;
    float aim_distance;
    void SetEnergyMode(int mode);
    void GetEnergyMode();
private:
    int energy_mode;

};
class EnergyBuff
{
public:
    void EnergyDetect(EnergyInfo &energy_buff,int &game_mode);
private:
    void EnergyPredict(EnergyInfo &energy_buff);
};




#endif // ENERGY_BUFF_H
