#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace Eigen;

/*-----------------------------------------------
    常数定义
------------------------------------------------*/
/* 数据转换比例因子 */
#define AS_XWGI 1.5258789063e-6 // XW-GI7681 陀螺仪数据转换比例因子
#define GS_XWGI 1.0850694444e-7 // XW-GI7681 陀螺仪数据转换比例因子
#define AS_NSC 2.0e-8           // NoVATel SPAN-100C 加速度计数据转换比例因子
#define GS_NSC 1.0e-9           // NoVATel SPAN-100C 陀螺仪数据转换比例因子

/* 惯导采样率 */
#define r_XWGI 100  // XW-GI7681 采样率100Hz
#define r_NSC 200   // NoVATel SPAN-100C 采样率200Hz

/* 常数 */
#define readtime 40.00          // 陀螺旋转读取时间
#define gravity 9.7936174       // 重力加速度9.7936174
#define increment 0.00018        // 角度积分增量阈值(度)
#define PAI 3.141592653589      // 圆周率
#define Deg 180.0/PAI           // 弧度转化为度
#define Rad PAI/180.0           // 度转化为弧度
#define we 7.292115e-5 * Deg    // 地球自转角速度(度每秒)
#define fai 30.531651244 * Rad  // 转台实验室纬度(rad)
#define circle 360.0            // 一圈360度

/* 设备类型 */
enum DeviceType { XWGI, NSC };      // XWGI为标定(calibration) NSC为对准(align)


/*-----------------------------------------------
    结构体定义
------------------------------------------------*/
/* GPS时间 */
struct GPSTIME
{
    int Week;   // 周
    double Second;  // 秒
    
    GPSTIME() { Week = 0; Second = 0; }
};

/* 加速度计数据 */
struct ACCDAT
{
    double X, Y, Z;

    ACCDAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};
/* 陀螺仪数据 */
struct GYRODAT
{
    double X, Y, Z;

    GYRODAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* 原始数据 */
struct RAWDAT
{
    GPSTIME time;   // 周 秒
    ACCDAT acc;     // 加速度计数据
    GYRODAT gyr;    // 陀螺仪数据（弧度）
};

/* 加速度计六位置标定 */
struct AccCaliError
{
    double bilimean[6][3];  // 实际测得的比力的平均值
    double bias[3];     // 零偏XYZ
    double scale[3];    // 比例因子误差XYZ
    double cross[3][2]; // 交轴耦合误差
    MatrixXd calibration;   // 加速度计误差矩阵
    
    AccCaliError()
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 6; j++) { bilimean[j][i] = 0.0; }
            bias[i] = 0.0; scale[i] = 0.0;
            for (int j = 0; j < 2; j++) { cross[i][j] = 0.0; }
        }
        calibration = MatrixXd(3, 4);
        calibration.setZero();
    }

    /* 提取误差矩阵中的元素 */
    // 设置零偏
    void SetBias() { for (int i = 0; i < 3; i++) { bias[i] = calibration(i, 3); } }

    // 设置比例因子
    void SetScale() { for (int i = 0; i < 3; i++) { scale[i] = calibration(i, i) - 1; } }

    // 设置交轴耦合误差
    void SetCross() 
    {
        for (int i = 0; i < 2; i++)
        {
            cross[0][i] = calibration(1 + i, 0);    // x对y和z
            cross[1][i] = calibration(i * 2, 1);    // y对x和z
            cross[2][i] = calibration(i, 2);        // z对x和y
        }
    }
};

/* 陀螺两位置标定 */
struct GyroCaliError
{
    GPSTIME prevtime;   // 上一个历元时间
    GYRODAT prevgyro;   // 上一个历元角速度
    GYRODAT angle_positive;   // 积分正角度
    GYRODAT angle_negative;   // 积分负角度
    double totalTpositive;    // 正旋转总用时
    double totalTnegative;    // 反旋转总用时
    double bias[3];     // 零偏XYZ(度/h)
    double scale[3];    // 比例因子XYZ

    GyroCaliError()
    {
        totalTpositive = 0.0; totalTnegative = 0.0;
        for (int i = 0; i < 3; i++)
        {
            bias[i] = 0.0; scale[i] = 0.0;
        }
    }
};

/* 粗对准结构体 */
struct ALIGNPOS
{
    GPSTIME time;   // 记录时间
    GPSTIME prevtime;   // 上一个历元的时间
    double billimean[3];   // 比力平均值
    double anglevelmean[3]; // 角速度平均值
    double yaw; // 俯仰角
    double pitch;   // 航向角
    double roll;    // 横滚角
    double biasA;   // 零偏主导的寻北误差
    double noiseA;  // 白噪声主导的寻北误差
    MatrixXd Cbn;   // 姿态矩阵

    ALIGNPOS()
    {
        yaw = 0.0; pitch = 0.0; roll = 0.0; biasA = 0.0; noiseA = 0.0;
        for (int i = 0; i < 3; i++)
        {
            billimean[i] = 0.0; anglevelmean[i] = 0.0;
        }
        Cbn = MatrixXd(3, 3);
        Cbn.setZero();
    }
    /* 提取姿态矩阵中的元素 */
    // 设置姿态角
    void SetPos()
    {
        double C31 = Cbn(2, 0); double C32 = Cbn(2, 1); double C33 = Cbn(2, 2);
        double C21 = Cbn(1, 0); double C11 = Cbn(0, 0);
        
        yaw = atan(-C31 / sqrt(C32 * C32 + C33 * C33)) * Deg; // 俯仰角(度)
        pitch = atan2(C21, C11) * Deg;    // 航向角(度)
        roll = atan2(C32, C33) * Deg;    // 横滚角(度)
    }
};


/*-----------------------------------------------
    文件操作函数
------------------------------------------------*/
bool ReadRawDataCali(ifstream& file, RAWDAT& Rawdata, DeviceType device);  // 读取一行标定有效原始数据
bool ReadRawDataAli(ifstream& file, RAWDAT& Rawdata, DeviceType device);    // 读取一行对准的有效原始数据

/*-----------------------------------------------
    标定函数
------------------------------------------------*/
void CalCaliMeanBili(ifstream& File, RAWDAT& Rawdata, int Place, AccCaliError& Error);   // 每个文件中加速度六个位置比力的平均值
void CaliAcc(AccCaliError& Error, ofstream& OutputFile);  // 加速度计的六位置标定
void CompenAcc(ifstream& File, RAWDAT& Rawdata, AccCaliError& Error, ofstream& OutputFile); // 对加速度原始数据进行补偿
void CalAngle(ifstream& File, RAWDAT& Rawdata, int mode, int Axis, GyroCaliError& Error, ofstream& FileSave);   // 积分计算旋转角度
void CaliGyro(int Axis, GyroCaliError& Error, int mode);   // 陀螺的两位置法标定
void CompenGyro(ifstream& File, RAWDAT& Rawdata, int mode, GyroCaliError& Error, ofstream& OutputFile);   // 对陀螺原始数据进行补偿
void CalGyrBias(ifstream& File, RAWDAT& Rawdata, GyroCaliError& Error, int mode); // 利用加速度计两位置数据计算陀螺零偏

/*-----------------------------------------------
    粗对准函数
------------------------------------------------*/
void CoarseAlign_EveryEpoch(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos1, ALIGNPOS& Pos2, ofstream& OutputFile);    // 每个历元粗对准
void CoarseAlign_EverySecond(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos1, ALIGNPOS& Pos2, ofstream& OutputFile); // 每秒粗对准
void CoarseAlign_Whole(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile);   // 整段数据粗对准
void CalTimeNoise(ifstream& File, RAWDAT& Rawdata, ofstream& OutputFile);   // 白噪声随平均时间变化
void CalCbn(ALIGNPOS& Pos);  // 计算姿态矩阵

/*-----------------------------------------------
    矩阵打印函数
------------------------------------------------*/
void Matrixprint(const MatrixXd Mat, const string name);
void Vectorprint(const VectorXd Mat, const string name);