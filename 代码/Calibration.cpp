/******* 标定 ********/
#include"IMU_Structs.h"

/*《加速度计》*/
/* 求加速度六个位置的比力平均值 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输入：第几个位置Place (int) 例：第五个位置 int place = 5
// 输出：加速度六位置比力平均值 例：AccCalError error.bilimean[]
void CalCaliMeanBili(ifstream& File, RAWDAT& Rawdata, int Place, AccCaliError& Error)
{
	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化加速度计三个方向的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double epochnum = 0.0;

	// 逐行读取文件
	while (ReadRawDataCali(File, Rawdata, calibration))
	{
		accmean[0] += Rawdata.acc.X;
		accmean[1] += Rawdata.acc.Y;
		accmean[2] += Rawdata.acc.Z;
		epochnum += 1.0;
	}
	for (int i = 0; i < 3; i++) { Error.bilimean[Place][i] = accmean[i] / epochnum; }
}

/* 加速度计六位置标定 */
// 输入：加速度六位置标定误差结构体，含有六位置比力平均值 例：AccCalError error
// 输出：加速度六位置标定误差结构体，得到误差矩阵 例：AccCalError error
// 输出：将加速度标定结果保存在文件中
void CaliAcc(AccCaliError& Error, ofstream& OutputFile)
{
	MatrixXd fmean = MatrixXd(3, 6);	// 实验测得比力
	fmean.setZero();
	MatrixXd freal = MatrixXd(4, 6);	// 已知输入比力
	freal.setZero();

	// 填充实验测得比力平均值
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			fmean(j, i) = Error.bilimean[i][j];
		}
	}

	// 填充已知输入
	for (int i = 0; i < 3; i++)
	{
		freal(i, i * 2) = gravity;
		freal(i, i * 2 + 1) = -gravity;
		
		freal(3, i * 2) = 1.0;	// 零偏
		freal(3, i * 2 + 1) = 1.0;
	}

	// 计算误差矩阵M
	Error.calibration = fmean * freal.transpose() * (freal * freal.transpose()).inverse();

	// 提取矩阵中的元素
	Error.SetBias();
	Error.SetCross();
	Error.SetScale();

	// 将结果保存在文件中
	OutputFile << fixed << setprecision(12);
	OutputFile << "%Bias_XYZ(m/s^2):\n" 
		<< Error.bias[0] << "," << Error.bias[1] << "," << Error.bias[2] << endl;

	OutputFile << "%Scale_XYZ:\n"
		<< Error.scale[0] << "," << Error.scale[1] << "," << Error.scale[2] << endl;

	OutputFile << "%Cross_XYZ:\n"
		<< 0 << "," << Error.cross[1][0] << "," << Error.cross[2][0] << endl
		<< Error.cross[0][0] << "," << 0 << "," << Error.cross[2][1] << endl
		<< Error.cross[0][1] << "," << Error.cross[1][1] << "," << 0 << endl;

	cout << "\n加速度计标定成功，结果已经保存在文件中！\n";
}

/* 补偿三轴加速度计 并 保存加速度计补偿前后的数据 */
void CompenAcc(ifstream& File, RAWDAT& Rawdata, AccCaliError& Error, ofstream& OutputFile)
{
	OutputFile << "%Time Compensation Raw(m/s^2):\n";

	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 逐行读取文件
	while (ReadRawDataCali(File, Rawdata, calibration))
	{
		Vector3d Com = Vector3d::Zero();	// 补偿后的
		Vector3d Raw = Vector3d::Zero();	// 原始数据

		MatrixXd M = MatrixXd(3, 3); M.setZero();	// 不含零偏的误差阵
		M = Error.calibration.block<3, 3>(0, 0);	// 拷贝
		Vector3d b = Vector3d::Zero();	// 零偏向量
		b = Error.calibration.col(3);  // 拷贝

		// 提取原始数据
		Raw(0) = Rawdata.acc.X;
		Raw(1) = Rawdata.acc.Y;
		Raw(2) = Rawdata.acc.Z;

		// 补偿数据
		Com = M.inverse() * (Raw - b);

		OutputFile << fixed << setprecision(2);
		OutputFile << Rawdata.time.Second << ",";

		OutputFile << fixed << setprecision(12);
		OutputFile << Com(0) << "," << Com(1) << "," << Com(2) << ","
			<< Raw(0) << "," << Raw(1) << "," << Raw(2) << endl;
	}
	cout << "\n已对加速度进行补偿，原始与补偿后的加速度数据已保存在文件！\n";
}


/*《陀螺》*/
/* 求陀螺单个轴的角速度积分 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输入：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
// 输入：正转反转标识mode 0为正转 1为反转
// 输入：三个轴Axis 0为X，1为Y，2为Z
// 输出：陀螺单个轴的正反旋转角度 例：GyroCaliError error.angle.X
void CalAngle(ifstream& File,RAWDAT& Rawdata, int mode, int Axis, GyroCaliError& Error, ofstream& FileSave)
{
	FileSave << "%Time X Y Z(Deg):\n";
	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化角度
	double angle[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;	// 历元数
	double totalT = 0.0;	// 总用时
	bool pprev = false, prev = false;
	bool normal = false;

	// 逐行读取文件并进行积分
	while (ReadRawDataCali(File, Rawdata, calibration))
	{
		// 是否开始的积分标志
		bool cur = false;

		// 获取时间差
		double dt = 0.0, dt_prev = 0.0;
		if (epochnum != 0)dt = (Rawdata.time.Week - Error.prevtime.Week) * 604800 + Rawdata.time.Second - Error.prevtime.Second;

		// 梯形法进行角速度积分
		double X = 0.0, Y = 0.0, Z = 0.0;
		if (epochnum > 0 && dt > 0.0)
		{
			X = 0.5 * (Rawdata.gyr.X + Error.prevgyro.X) * dt * Deg;
			Y = 0.5 * (Rawdata.gyr.Y + Error.prevgyro.Y) * dt * Deg;
			Z = 0.5 * (Rawdata.gyr.Z + Error.prevgyro.Z) * dt * Deg;

			FileSave << fixed << setprecision(12);
			FileSave << Rawdata.time.Second << "," << X << "," << Y << "," << Z << endl;

			// 获取轴信息
			double add = 0.0;
			switch (Axis)
			{
			case 0: { add = X; break; }
			case 1: { add = Y; break; }
			case 2: { add = Z; break; }
			}
			// 判断是否达到阈值
			if (fabs(add) >= increment) { cur = true; }	// 大于阈值，开始积分

			// 正常积分
			if (cur == true && fabs(totalT - readtime) > 1e-3)
			{
				angle[0] += X;
				angle[1] += Y;
				angle[2] += Z;
				totalT += dt;
				normal = true;
			}
			// 正常积分之后，若未到达指定积分时间readtime，则继续积分
			else if (normal == true && fabs(totalT - readtime) > 1e-3)
			{
				//cout << Rawdata.time.Second << endl;
				angle[0] += X;
				angle[1] += Y;
				angle[2] += Z;
				totalT += dt;
			}

			if (fabs(totalT - readtime) < 1e-3)
			{
				int j = 0;
			}
		}
		dt_prev = dt;

		// 更新角速度，时间
		Error.prevgyro = Rawdata.gyr;
		Error.prevtime = Rawdata.time;

		epochnum += 1.0;
	}

	// 存储到结构体
	if (mode == 0)
	{
		Error.totalTpositive = totalT;
		for (int i = 0; i < 3; i++)
		{
			Error.angle_positive.X = angle[0];
			Error.angle_positive.Y = angle[1];
			Error.angle_positive.Z = angle[2];
		}
	}
	else if (mode == 1)
	{
		Error.totalTnegative = totalT;
		for (int i = 0; i < 3; i++)
		{
			Error.angle_negative.X = angle[0];
			Error.angle_negative.Y = angle[1];
			Error.angle_negative.Z = angle[2];
		}
	}
}

/* 陀螺的两位置法标定 */
// 输入：三个轴Axis 0为X，1为Y，2为Z
// 输入：模式mode 0时计算陀螺零偏与比例因子 1仅计算零偏
// 输出：单个轴的陀螺零偏和比例因子
// 输出：将陀螺零偏和比例因子保存在文件中
void CaliGyro(int Axis, GyroCaliError& Error, int mode)
{
	double totalT = (Error.totalTnegative + Error.totalTpositive) / 2.0;	// 总用时为正转和反转的平均

	// 零偏
	switch (Axis)
	{
	case 0:
		if (mode == 0) {
			Error.bias[0] = (Error.angle_positive.X + Error.angle_negative.X) / (2.0 * totalT) - we * sin(fai);	// 零偏
			Error.scale[0] = abs((Error.angle_positive.X - Error.angle_negative.X)) / (2.0 * circle) - 1.0;		// 比例因子
		}
		else if (mode == 1)Error.bias[0] = (Error.angle_positive.X + Error.angle_negative.X) / (2.0);
		break;
	case 1:
		if (mode == 0) {
			Error.bias[1] = (Error.angle_positive.Y + Error.angle_negative.Y) / (2.0 * totalT) - we * sin(fai);
			Error.scale[1] = abs((Error.angle_positive.Y - Error.angle_negative.Y)) / (2.0 * circle) - 1.0;
		}
		else if (mode == 1)Error.bias[1] = (Error.angle_positive.Y + Error.angle_negative.Y) / (2.0);
		break;
	case 2:
		if (mode == 0) {
			Error.bias[2] = (Error.angle_positive.Z + Error.angle_negative.Z) / (2.0 * totalT) - we * sin(fai);
			Error.scale[2] = abs((Error.angle_positive.Z - Error.angle_negative.Z)) / (2.0 * circle) - 1.0;
		}
		else if (mode == 1)Error.bias[2] = (Error.angle_positive.Z + Error.angle_negative.Z) / (2.0);
		break;
	}
	for (int i = 0; i < 3; i++) { Error.bias[i] = Error.bias[i] * Deg * 3600.0; }	// 度每小时
}

/* 利用加速度计数据标定陀螺零偏 */
// 输入：输入文件流
// 输入：Rawdata
// 输入：模式：mode 0为positive 1为negative
// 输出：Error.positive Error.negative
void CalGyrBias(ifstream& File, RAWDAT& Rawdata, GyroCaliError& Error, int mode)
{
	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 初始化陀螺仪三个方向的平均值
	double gyrmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 逐行读取文件
	while (ReadRawDataCali(File, Rawdata, calibration))
	{
		gyrmean[0] += Rawdata.gyr.X;
		gyrmean[1] += Rawdata.gyr.Y;
		gyrmean[2] += Rawdata.gyr.Z;

		epochnum += 1.0;
	}
	if (mode == 0)
	{
		Error.angle_positive.X = gyrmean[0] / epochnum;
		Error.angle_positive.Y = gyrmean[1] / epochnum;
		Error.angle_positive.Z = gyrmean[2] / epochnum;
	}
	if (mode == 1)
	{
		Error.angle_negative.X = gyrmean[0] / epochnum;
		Error.angle_negative.Y = gyrmean[1] / epochnum;
		Error.angle_negative.Z = gyrmean[2] / epochnum;
	}
}


/* 补偿陀螺误差 并 输出原始与补偿后数据 */
void CompenGyro(ifstream& File, RAWDAT& Rawdata, int mode, GyroCaliError& Error, ofstream& OutputFile)
{
	OutputFile << "%Time Compensation_XYZ Raw_XYZ(Rad/s):\n";
	// 设备类型
	DeviceType calibration = XWGI;	// 标定 设备为XW-GI7681
	DeviceType align = NSC;			// 对准 设备为NoVATel SPAN-100C

	// 逐行读取文件
	while (ReadRawDataCali(File, Rawdata, calibration))
	{
		Vector3d Com = Vector3d::Zero();	// 补偿后的
		Vector3d Raw = Vector3d::Zero();	// 原始数据

		MatrixXd M = MatrixXd(3, 3); M.setZero();	// 不含零偏的误差阵
		for (int i = 0; i < 3; i++)M(i, i) = Error.scale[i] + 1.0;	// 提取比例因子


		Vector3d b = Vector3d::Zero();	// 零偏向量
		for (int i = 0; i < 3; i++)b(i) = Error.bias[i] * Rad / 3600.0;	// 提取零偏(°/h->rad/s)

		// 提取原始数据
		Raw(0) = Rawdata.gyr.X;
		Raw(1) = Rawdata.gyr.Y;
		Raw(2) = Rawdata.gyr.Z;

		// 补偿数据
		Com = M.inverse() * (Raw - b);

		OutputFile << fixed << setprecision(2);
		OutputFile << Rawdata.time.Second << ",";
		OutputFile << fixed << setprecision(12);

		switch (mode)
		{
		case 0:
			OutputFile << Com(0) - we * sin(fai) * Rad << "," << Raw(0) << endl;
			break;
		case 1:
			OutputFile << Com(1) - we * sin(fai) * Rad << "," << Raw(1) << endl;
			break;
		case 2:
			OutputFile << Com(2) - we * sin(fai) * Rad << "," << Raw(2) << endl;
			break;
		}
	}
	cout << "\n已对角速度进行补偿，原始与补偿后的角速度数据已保存在文件！\n";
}