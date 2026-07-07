# IMU 标定与初始对准实验

本仓库整理了惯性导航原理实验中的 IMU 传感器标定与静态初始对准代码。项目基于 C++ 和 Eigen 实现，包含加速度计六位置标定、陀螺仪正反向旋转标定、静态粗对准，以及 MATLAB 结果绘图脚本和实验报告。

仓库中原始数据与结果以压缩包形式保存：运行程序前需要先在仓库根目录解压 `Data.zip` 和 `Result.zip`，解压后程序会按 `Data/` 和 `Result/` 的相对路径读取和写出文件。

## 快速使用

1. 解压数据与结果目录：

```powershell
Expand-Archive .\Data.zip -DestinationPath .
Expand-Archive .\Result.zip -DestinationPath .
```

2. 编译 C++ 程序：

```powershell
g++ -std=c++11 .\代码\main.cpp .\代码\Calibration.cpp .\代码\Align.cpp .\代码\ReadFile.cpp .\代码\MatrixPrint.cpp -I <Eigen路径> -o imu_alignment_calibration.exe
```

3. 运行：

```powershell
.\imu_alignment_calibration.exe
```

程序启动后输入模式编号：

```text
0  加速度计六位置标定
1  陀螺仪旋转标定
2  静态初始对准
```

## 主要功能

- 加速度计标定：读取六个静态位置的 XW-GI7681 数据，估计零偏、比例因子和交叉耦合误差。
- 陀螺仪标定：读取 X/Y/Z 三轴正反向 360 度旋转数据，积分角速度并估计零偏和比例因子。
- 静态初始对准：基于 NoVATel SPAN-100C 静态数据计算整段、逐秒和逐历元姿态角。
- 误差补偿输出：保存原始观测与补偿后观测，便于后续 MATLAB 对比。
- MATLAB 绘图：`Result/Figure_Matlab/` 中包含加速度计、陀螺仪和对准结果绘图脚本。

## 项目结构

```text
.
├─ Data.zip                       # 原始 IMU 数据压缩包，运行前解压
├─ Result.zip                     # 已生成结果与 MATLAB 脚本压缩包，运行前解压
├─ 代码/
│  ├─ main.cpp                    # 交互式入口，选择 0/1/2 三种处理模式
│  ├─ IMU_Structs.h               # 常量、数据结构和函数声明
│  ├─ Calibration.cpp             # 加速度计/陀螺仪标定与补偿
│  ├─ Align.cpp                   # 静态粗对准与噪声统计
│  ├─ ReadFile.cpp                # ASC 数据读取
│  └─ MatrixPrint.cpp             # 矩阵/向量打印工具
├─ 实验报告/
│  ├─ 实验报告.pdf
│  └─ 图例.pptx
├─ 实验指导书/
│  ├─ 惯性导航实验指导书.pdf
│  └─ 【V2】初始对准实验与标定试验数据解码与转换方法.docx
└─ README.md
```

解压后的主要目录：

```text
Data/
├─ Calibration/                   # x/y/z 六位置与三轴正反转数据
└─ Align/                         # Align_30min.txt 静态对准数据

Result/Figure_Matlab/
├─ AccCali/                       # 加速度计标定结果
├─ GyrCali/                       # 陀螺仪标定结果
├─ Align/                         # 初始对准结果
├─ AccCali_fig.m
├─ GyrCali_fig.m
├─ Align_fig.m
└─ main.m
```

## 代码模块

| 文件 | 说明 |
| --- | --- |
| `main.cpp` | 根据输入模式调度加速度计标定、陀螺仪标定或初始对准 |
| `Calibration.cpp` | `CaliAcc`、`CaliGyro`、`CompenAcc`、`CompenGyro` 等标定与补偿函数 |
| `Align.cpp` | `CoarseAlign_Whole`、`CoarseAlign_EverySecond`、`CoarseAlign_EveryEpoch`、`CalTimeNoise` |
| `ReadFile.cpp` | 标定数据和对准数据的逐行读取与单位转换 |
| `IMU_Structs.h` | GPS 时间、IMU 原始观测、标定误差、姿态结果等结构体定义 |

## 输出结果

```text
Result/Figure_Matlab/AccCali/AccCaliError.txt
Result/Figure_Matlab/AccCali/*_Raw_Com.txt
Result/Figure_Matlab/GyrCali/GyroCalibration.txt
Result/Figure_Matlab/GyrCali/*_Angle.txt
Result/Figure_Matlab/GyrCali/*_Raw_Com.txt
Result/Figure_Matlab/Align/Align_Whole.txt
Result/Figure_Matlab/Align/Align_Second.txt
Result/Figure_Matlab/Align/Align_Epoch.txt
Result/Figure_Matlab/Align/Align_Deviation.txt
```

MATLAB 可视化：

```matlab
cd Result/Figure_Matlab
main
```

## 环境要求

- C++11 编译器
- Eigen3
- MATLAB，用于结果绘图
- Windows 路径下运行最省事，因为源码中的数据路径使用 `Data\...` 和 `Result\...`

## 报告

```text
实验报告/实验报告.pdf
```

## 作者

GYH-WHU
