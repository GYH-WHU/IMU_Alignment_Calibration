#include "IMU_Structs.h"

int main()
{
	int mode = -1;
	cout << "0加速度计标定 1陀螺仪标定 2粗对准\n" << "请输入你需要解算的内容：";
	cin >> mode;
	if (!(mode == 0 || mode == 1 || mode == 2))cerr << "\n未识别解算内容“" << mode << "” 程序结束！\n";

	switch (mode){

	/* 标定加速度计 */
	case 0: {
		// 标定文件路径
		string filename_1 = "Data\\Calibration\\x_up_3min.ASC";
		string filename_2 = "Data\\Calibration\\x_down_3min.ASC";
		string filename_3 = "Data\\Calibration\\y_up_3min.ASC";
		string filename_4 = "Data\\Calibration\\y_down_3min.ASC";
		string filename_5 = "Data\\Calibration\\z_up_3min.ASC";
		string filename_6 = "Data\\Calibration\\z_down_3min.ASC";
		string filename_c1 = "Data\\Calibration\\x_up_3min.ASC";
		string filename_c2 = "Data\\Calibration\\x_down_3min.ASC";
		string filename_c3 = "Data\\Calibration\\y_up_3min.ASC";
		string filename_c4 = "Data\\Calibration\\y_down_3min.ASC";
		string filename_c5 = "Data\\Calibration\\z_up_3min.ASC";
		string filename_c6 = "Data\\Calibration\\z_down_3min.ASC";

		string filesave = "Result\\Figure_Matlab\\AccCali\\AccCaliError.txt";
		string filesave_c1 = "Result\\Figure_Matlab\\AccCali\\x_up_Raw_Com.txt";
		string filesave_c2 = "Result\\Figure_Matlab\\AccCali\\x_down_Raw_Com.txt";
		string filesave_c3 = "Result\\Figure_Matlab\\AccCali\\y_up_Raw_Com.txt";
		string filesave_c4 = "Result\\Figure_Matlab\\AccCali\\y_down_Raw_Com.txt";
		string filesave_c5 = "Result\\Figure_Matlab\\AccCali\\z_up_Raw_Com.txt";
		string filesave_c6 = "Result\\Figure_Matlab\\AccCali\\z_down_Raw_Com.txt";

		RAWDAT rawdata;
		AccCaliError error;

		// 打开文件流，检查文件是否存在
		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
		ofstream file(filesave), file_s1(filesave_c1), file_s2(filesave_c2), file_s3(filesave_c3), file_s4(filesave_c4), file_s5(filesave_c5), file_s6(filesave_c6);
		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 || file)) { cerr << "\nCannot open the file to read rawdata!\n"; return -1; }
		if (!(file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6 || file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "\nCannot open the file to read rawdata!\n"; return -1; }

		// 六位置法标定加速度计
		CalCaliMeanBili(file_1, rawdata, 0, error);	// 求平均
		CalCaliMeanBili(file_2, rawdata, 1, error);
		CalCaliMeanBili(file_3, rawdata, 2, error);
		CalCaliMeanBili(file_4, rawdata, 3, error);
		CalCaliMeanBili(file_5, rawdata, 4, error);
		CalCaliMeanBili(file_6, rawdata, 5, error);

		CaliAcc(error, file);	// 求误差矩阵

		// 补偿误差
		CompenAcc(file_c1, rawdata, error, file_s1);
		CompenAcc(file_c2, rawdata, error, file_s2);
		CompenAcc(file_c3, rawdata, error, file_s3);
		CompenAcc(file_c4, rawdata, error, file_s4);
		CompenAcc(file_c5, rawdata, error, file_s5);
		CompenAcc(file_c6, rawdata, error, file_s6);

		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();	// 关闭文件流
		file.close();
		file_c1.close(); file_c2.close(); file_c3.close(); file_c4.close(); file_c5.close(); file_c6.close();
		file_s1.close(); file_s2.close(); file_s3.close(); file_s4.close(); file_s5.close(); file_s6.close();

		break;
	}

	/* 标定陀螺 */
	case 1: {
		// 标定文件路径
		string filename_1 = "Data\\Calibration\\x+360.ASC";
		string filename_2 = "Data\\Calibration\\x-360.ASC";
		string filename_3 = "Data\\Calibration\\y+360.ASC";
		string filename_4 = "Data\\Calibration\\y-360.ASC";
		string filename_5 = "Data\\Calibration\\z+360.ASC";
		string filename_6 = "Data\\Calibration\\z-360.ASC";
		string filename_c1 = "Data\\Calibration\\x+360.ASC";
		string filename_c2 = "Data\\Calibration\\x-360.ASC";
		string filename_c3 = "Data\\Calibration\\y+360.ASC";
		string filename_c4 = "Data\\Calibration\\y-360.ASC";
		string filename_c5 = "Data\\Calibration\\z+360.ASC";
		string filename_c6 = "Data\\Calibration\\z-360.ASC";
		string filename_acc1 = "Data\\Calibration\\x_up_3min.ASC";
		string filename_acc2 = "Data\\Calibration\\x_down_3min.ASC";
		string filename_acc3 = "Data\\Calibration\\y_up_3min.ASC";
		string filename_acc4 = "Data\\Calibration\\y_down_3min.ASC";
		string filename_acc5 = "Data\\Calibration\\z_up_3min.ASC";
		string filename_acc6 = "Data\\Calibration\\z_down_3min.ASC";

		string filesave = "Result\\Figure_Matlab\\GyrCali\\GyroCalibration.txt";
		string filesave1 = "Result\\Figure_Matlab\\GyrCali\\x+360_Angle.txt";
		string filesave2 = "Result\\Figure_Matlab\\GyrCali\\x-360_Angle.txt";
		string filesave3 = "Result\\Figure_Matlab\\GyrCali\\y+360_Angle.txt";
		string filesave4 = "Result\\Figure_Matlab\\GyrCali\\y-360_Angle.txt";
		string filesave5 = "Result\\Figure_Matlab\\GyrCali\\z+360_Angle.txt";
		string filesave6 = "Result\\Figure_Matlab\\GyrCali\\z-360_Angle.txt";
		string filesave_s1 = "Result\\Figure_Matlab\\GyrCali\\x+360_Raw_Com.txt";
		string filesave_s2 = "Result\\Figure_Matlab\\GyrCali\\x-360_Raw_Com.txt";
		string filesave_s3 = "Result\\Figure_Matlab\\GyrCali\\y+360_Raw_Com.txt";
		string filesave_s4 = "Result\\Figure_Matlab\\GyrCali\\y-360_Raw_Com.txt";
		string filesave_s5 = "Result\\Figure_Matlab\\GyrCali\\z+360_Raw_Com.txt";
		string filesave_s6 = "Result\\Figure_Matlab\\GyrCali\\z-360_Raw_Com.txt";

		RAWDAT rawdata;
		GyroCaliError error_x, error_y, error_z;

		// 打开文件流，检查文件是否存在
		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
		ifstream file_acc1(filename_acc1), file_acc2(filename_acc2), file_acc3(filename_acc3), file_acc4(filename_acc4), file_acc5(filename_acc5), file_acc6(filename_acc6);
		ofstream file(filesave), file1(filesave1), file2(filesave2), file3(filesave3), file4(filesave4), file5(filesave5), file6(filesave6);
		ofstream file_s1(filesave_s1), file_s2(filesave_s2), file_s3(filesave_s3), file_s4(filesave_s4), file_s5(filesave_s5), file_s6(filesave_s6);
		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 || file)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file_acc1 || file_acc2 || file_acc3 || file_acc4 || file_acc5 || file_acc6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }

		// 积分计算三个轴正转反转旋转角度
		CalAngle(file_1, rawdata, 0, 0, error_x, file1);
		CalAngle(file_2, rawdata, 1, 0, error_x, file2);
		CalAngle(file_3, rawdata, 0, 1, error_y, file3);
		CalAngle(file_4, rawdata, 1, 1, error_y, file4);
		CalAngle(file_5, rawdata, 0, 2, error_z, file5);
		CalAngle(file_6, rawdata, 1, 2, error_z, file6);

		CaliGyro(0, error_x, 0);	// 标定求比例因子
		CaliGyro(1, error_y, 0);
		CaliGyro(2, error_z, 0);

		CalGyrBias(file_acc1, rawdata, error_x, 0);
		CalGyrBias(file_acc2, rawdata, error_x, 1);
		CalGyrBias(file_acc3, rawdata, error_y, 0);
		CalGyrBias(file_acc4, rawdata, error_y, 1);
		CalGyrBias(file_acc5, rawdata, error_z, 0);
		CalGyrBias(file_acc6, rawdata, error_z, 1);

		CaliGyro(0, error_x, 1);	// 标定求零偏
		CaliGyro(1, error_y, 1);
		CaliGyro(2, error_z, 1);
		cout << "\n陀螺标定成功，结果已保存在文件中！\n";

		// 补偿陀螺原始数据
		GyroCaliError error;	// 将三轴的误差项提取出来
		error.bias[0] = error_x.bias[0]; error.bias[1] = error_y.bias[1]; error.bias[2] = error_z.bias[2];
		error.scale[0] = error_x.scale[0]; error.scale[1] = error_y.scale[1]; error.scale[2] = error_z.scale[2];

		CompenGyro(file_c1, rawdata, 0, error, file_s1);
		CompenGyro(file_c2, rawdata, 0, error, file_s2);
		CompenGyro(file_c3, rawdata, 1, error, file_s3);
		CompenGyro(file_c4, rawdata, 1, error, file_s4);
		CompenGyro(file_c5, rawdata, 2, error, file_s5);
		CompenGyro(file_c6, rawdata, 2, error, file_s6);

		// 保存误差
		file << fixed << setprecision(12);
		file << "%Bias_XYZ(deg/h):\n"
			<< error_x.bias[0] << "," << error_y.bias[1] << "," << error_z.bias[2] << endl;
		file << "%Scale(XYZ):\n"
			<< error_x.scale[0] << "," << error_y.scale[1] << "," << error_z.scale[2];

		// 关闭文件流
		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();
		file.close(); file1.close(); file2.close(); file3.close(); file4.close(); file5.close(); file6.close();
		break;
	}

	/* 粗对准 */
	case 2:
	{
		// 打开数据文件
		string filename1 = "Data\\Align\\Align_30min.ASC";
		string filename2 = "Data\\Align\\Align_30min.ASC";
		string filename3 = "Data\\Align\\Align_30min.ASC";
		string filename4 = "Data\\Align\\Align_30min.ASC";
		
		// 打开数据保存文件
		string filename_save1 = "Result\\Figure_Matlab\\Align\\Align_Whole.txt";
		string filename_save2 = "Result\\Figure_Matlab\\Align\\Align_Second.txt";
		string filename_save3 = "Result\\Figure_Matlab\\Align\\Align_Epoch.txt";
		string filename_save4 = "Result\\Figure_Matlab\\Align\\Align_Deviation.txt";

		// 创建结构体
		RAWDAT rawdata;
		ALIGNPOS pos1, pos2, pos3;

		// 打开文件流，检查文件是否存在
		ifstream file1(filename1), file2(filename2), file3(filename3), file4(filename4);
		ofstream file_save1(filename_save1), file_save2(filename_save2), file_save3(filename_save3), file_save4(filename_save4);
		if (!file1 || !file_save1 || !file_save2 || !file_save3 || !file_save4) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }

		// 计算姿态角
		CoarseAlign_Whole(file1, rawdata, pos1, file_save1);	// 计算整段数据的姿态角并保存
		CoarseAlign_EverySecond(file2, rawdata, pos2, pos1, file_save2);	// 计算每一秒的平均姿态角并保存
		CoarseAlign_EveryEpoch(file3, rawdata, pos3, pos1, file_save3);	// 计算每个历元的姿态角并保存

		CalTimeNoise(file4, rawdata, file_save4);	// 计算白噪声随时间变化大小

		file1.close(); file2.close(); file3.close(); file4.close();	// 关闭文件流
		file_save1.close();
		file_save2.close();
		file_save3.close();
		file_save4.close();

		break;
	}
	}
	return 0;
}