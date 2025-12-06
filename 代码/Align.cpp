/******* 对准 ********/
#include"IMU_Structs.h"
using namespace std;

/* 每个历元粗对准 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：姿态 例：ALIGNPOS pos.yaw
// 输出：姿态文件
void CoarseAlign_EveryEpoch(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos1, ALIGNPOS& Pos2, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;
	double rmse[3] = { 0.0,0.0,0.0 };

	// 逐行读取文件
	while (ReadRawDataAli(File, Rawdata, align))
	{
		// 提取时间
		Pos1.time = Rawdata.time;

		Pos1.billimean[0] = Rawdata.acc.X;
		Pos1.billimean[1] = Rawdata.acc.Y;
		Pos1.billimean[2] = Rawdata.acc.Z;

		Pos1.anglevelmean[0] = Rawdata.gyr.X;	// 弧度
		Pos1.anglevelmean[1] = Rawdata.gyr.Y;	// 弧度
		Pos1.anglevelmean[2] = Rawdata.gyr.Z;	// 弧度
		epochnum++;

		// 计算姿态角
		CalCbn(Pos1);
		rmse[0] += pow(Pos1.yaw - Pos2.yaw, 2);
		rmse[1] += pow(Pos1.pitch - Pos2.pitch, 2);
		rmse[2] += pow(Pos1.roll - Pos2.roll, 2);

		OutputFile << fixed << setprecision(12);
		OutputFile << Pos1.time.Second << ",";
		OutputFile << Pos1.yaw << "," << Pos1.pitch << "," << Pos1.roll << endl;
	}
	for (int i = 0; i < 3; i++)
	{
		rmse[i] = sqrt(rmse[i] / epochnum);
	}
	printf("\n每历元计算的姿态角的rms为(yaw pitch roll)：%.6f %.6f %.6f", rmse[0], rmse[1], rmse[2]);
	cout << "\n每历元的姿态角求解成功，结果已保存在文件中！\n";
}

/* 每秒粗对准 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：姿态 例：ALIGNPOS pos.yaw
// 输出：姿态文件
void CoarseAlign_EverySecond(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos1, ALIGNPOS& Pos2, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";
	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0, T = 0.0;
	double totaltime = 0.0;
	double dt = 0.0;
	double rmse[3] = { 0.0, 0.0, 0.0 };

	// 逐行读取文件
	while (ReadRawDataAli(File, Rawdata, align))
	{
		// 提取时间
		Pos1.time = Rawdata.time;
		if (epochnum != 0)dt = (Pos1.time.Week - Pos1.prevtime.Week) * 604800 + Pos1.time.Second - Pos1.prevtime.Second;
		totaltime += dt;
		// 当读取时间小于1秒时
		if (fabs(totaltime - 1.0) > 1e-3)
		{
			accmean[0] += Rawdata.acc.X;
			accmean[1] += Rawdata.acc.Y;
			accmean[2] += Rawdata.acc.Z;

			avelmean[0] += Rawdata.gyr.X;	// 弧度
			avelmean[1] += Rawdata.gyr.Y;	// 弧度
			avelmean[2] += Rawdata.gyr.Z;	// 弧度

			epochnum += 1.0;
		}
		// 等于一秒时
		else if (fabs(totaltime - 1.0) < 1e-3)
		{
			// 求平均
			for (int i = 0; i < 3; i++)
			{
				Pos1.billimean[i] = accmean[i] / epochnum;
				Pos1.anglevelmean[i] = avelmean[i] / epochnum;
			}
			// 计算姿态角
			CalCbn(Pos1);
			rmse[0] += pow(Pos1.yaw - Pos2.yaw, 2);
			rmse[1] += pow(Pos1.pitch - Pos2.pitch, 2);
			rmse[2] += pow(Pos1.roll - Pos2.roll, 2);

			T++;

			OutputFile << fixed << setprecision(0) << T << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << Pos1.yaw << "," << Pos1.pitch << "," << Pos1.roll << endl;

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				avelmean[i] = 0.0;
			}
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元
		}
		// 将该历元时间保存
		Pos1.prevtime = Pos1.time;
	}
	for (int i = 0; i < 3; i++)
	{
		rmse[i] = sqrt(rmse[i] / T);
	}
	printf("\n每秒平均计算的姿态角的rmse为(yaw pitch roll)：%.6f %.6f %.6f", rmse[0], rmse[1], rmse[2]);
	cout << "\n每秒平均的姿态角求解成功，结果已保存在文件中！\n";
}

/* 整段粗对准 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输出：姿态 例：ALIGNPOS pos.yaw
// 输出：姿态文件
void CoarseAlign_Whole(ifstream& File, RAWDAT& Rawdata, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 逐行读取文件
	while (ReadRawDataAli(File, Rawdata, align))
	{
		accmean[0] += Rawdata.acc.X;
		accmean[1] += Rawdata.acc.Y;
		accmean[2] += Rawdata.acc.Z;

		avelmean[0] += Rawdata.gyr.X;	// 弧度
		avelmean[1] += Rawdata.gyr.Y;	// 弧度
		avelmean[2] += Rawdata.gyr.Z;	// 弧度

		epochnum += 1.0;
	}
	// 求平均
	for (int i = 0; i < 3; i++)
	{
		Pos.billimean[i] = accmean[i] / epochnum;
		Pos.anglevelmean[i] = avelmean[i] / epochnum;
	}

	// 计算姿态角
	CalCbn(Pos);

	OutputFile << fixed << setprecision(12);
	OutputFile << Pos.yaw << "," << Pos.pitch << "," << Pos.roll << endl;

	cout << "\n整段数据的平均姿态角求解成功，结果已保存在文件中！\n";
}

/* 计算随平均时间变化的白噪声误差 */
void CalTimeNoise(ifstream& File, RAWDAT& Rawdata, ofstream& OutputFile)
{
	OutputFile << "%Bias(deg): " << fixed << setprecision(12) << Deg * ((0.05 / 3600.0) / (we * cos(fai))) << endl;
	OutputFile << "%Time(s),WhiteNoise(deg):\n";

	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化加速度计和角速度的平均值
	double t0 = 0.0, totalt = 0;
	double epochnum = 0.0;

	// 逐行读取文件
	while (ReadRawDataAli(File, Rawdata, align))
	{
		if (epochnum == 0) { t0 = Rawdata.time.Week * 604800 + Rawdata.time.Second;}	// 取初始时间
		epochnum++;
		totalt = Rawdata.time.Week * 604800 + Rawdata.time.Second - t0;
		if (epochnum == 1)continue;	// 避免totalt为0

		OutputFile << fixed << setprecision(4);
		OutputFile << totalt << ",";
		OutputFile << fixed << setprecision(12);
		OutputFile << Deg * ((0.005 / 3600.0) / ((we * cos(fai)) * sqrt(totalt))) << endl;
	}

	cout << "\n白噪声误差随时间变化的结果计算成功，结果已保存在文件中！\n";
}

/* 计算姿态矩阵 */
void CalCbn(ALIGNPOS& Pos)
{
	// 将平均比力、角速度转化为观测向量
	Vector3d gb = Vector3d::Zero(3, 1);	// 比力
	Vector3d wb = Vector3d::Zero(3, 1);	// 角速度
	Vector3d v = Vector3d::Zero(3, 1);		// 比力和角速度的叉乘

	for (int i = 0; i < 3; i++)
	{
		gb(i) = -Pos.billimean[i];
		wb(i) = Pos.anglevelmean[i];
	}
	v = gb.cross(wb);	// 向量叉乘
	
	// 定义输入向量
	Vector3d gn(0.0, 0.0, gravity);	// 重力输入量
	Vector3d wn(we * cos(fai), 0.0, -we * sin(fai));	// 角速度输入量

	// 正交化与归一化
	Vector3d wg = gb.normalized();	// 归一化观测比力
	Vector3d ww = v.normalized();
	Vector3d wgw = (v.cross(gb)).normalized();

	Vector3d vg = gn.normalized();	// 归一化输入重力
	Vector3d vw = (gn.cross(wn)).normalized();
	Vector3d vgw = ((gn.cross(wn)).cross(gn)).normalized();

	// 求解姿态矩阵
	MatrixXd V = MatrixXd(3, 3); V.setZero();	// 输入矩阵
	V.col(0) = vg; V.col(1) = vw; V.col(2) = vgw;
	MatrixXd W = MatrixXd(3, 3); W.setZero();	// 观测矩阵
	W.col(0) = wg; W.col(1) = ww; W.col(2) = wgw;

	Pos.Cbn = V * W.transpose();	// 姿态矩阵

	Pos.SetPos();	// 设置姿态角
}