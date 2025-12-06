#include"IMU_Structs.h"

/* 读取一行标定的有效原始数据 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输出：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
bool ReadRawDataCali(ifstream& file, RAWDAT& Rawdata, DeviceType device)
{
	// 采样率，加速度计、陀螺仪数据转换比例因子
	double rate = 0.0, convert_a = 0.0, convert_g = 0.0;
	if (device == XWGI) { rate = r_XWGI; convert_a = AS_XWGI; convert_g = GS_XWGI; }	// 标定 Calibration
	else if (device == NSC) { rate = r_NSC; convert_a = AS_NSC; convert_g = GS_NSC; }	// 对准 Align
	else return false;

	// 读取文件数据
	string line;
	if(getline(file, line))
	{
		if (line.empty())return false;	// 行为空时继续循环

		stringstream ss(line);	// 使用字符串流 逐个字段解析

		string part;
		char delimiter;

		// 跳过%RAWIMUSA,2387,558507.850;
		getline(ss, part, ';');

		// 读取周 周内秒
		getline(ss, part, ',');	// 周
		Rawdata.time.Week = stoi(part);
		getline(ss, part, '.');	// 周内秒
		Rawdata.time.Second = stod(part);
		getline(ss, part, ',');
		Rawdata.time.Second += stod(part) * 1e-9;

		// 跳过00000077,
		getline(ss, part, ','); 

		// 读取加速度计数据
		getline(ss, part, ',');	// Z
		Rawdata.acc.Z = stod(part) * convert_a * rate;
		getline(ss, part, ',');	// -Y
		Rawdata.acc.Y = -stod(part) * convert_a * rate;
		getline(ss, part, ',');	// X
		Rawdata.acc.X = stod(part) * convert_a * rate;

		// 读取陀螺仪数据
		getline(ss, part, ',');	// Z(弧度)
		Rawdata.gyr.Z = stod(part) * convert_g * rate;
		getline(ss, part, ',');	// -Y(弧度)
		Rawdata.gyr.Y = -stod(part) * convert_g * rate;
		getline(ss, part, '*');	// X(弧度)
		Rawdata.gyr.X = stod(part) * convert_g * rate;

		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}

/* 读取一行对准的有效原始数据 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输出：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
bool ReadRawDataAli(ifstream& file, RAWDAT& Rawdata, DeviceType device)
{
	// 采样率，加速度计、陀螺仪数据转换比例因子
	double rate = 0.0, convert_a = 0.0, convert_g = 0.0;
	if (device == XWGI) { rate = r_XWGI; convert_a = AS_XWGI; convert_g = GS_XWGI; }	// 标定 Calibration
	else if (device == NSC) { rate = r_NSC; convert_a = AS_NSC; convert_g = GS_NSC; }	// 对准 Align
	else return false;

	// 读取文件数据
	string line;
	if (getline(file, line))
	{
		if (line.empty())return false;	// 行为空时继续循环

		stringstream ss(line);	// 使用字符串流 逐个字段解析

		string part;
		char delimiter;

		// 跳过%RAWIMUSA,2387,558507.850;
		getline(ss, part, ';');

		// 跳过04,26,
		getline(ss, part, ',');
		getline(ss, part, ',');

		// 读取周 周内秒
		getline(ss, part, ',');	// 周
		Rawdata.time.Week = stoi(part);
		getline(ss, part, '.');	// 周内秒
		Rawdata.time.Second = stod(part);
		getline(ss, part, ',');
		Rawdata.time.Second += stod(part) * 1e-9;

		// 跳过2e480000,
		getline(ss, part, ',');

		// 读取加速度计数据
		getline(ss, part, ',');	// Z
		Rawdata.acc.Z = stod(part) * convert_a * rate;
		getline(ss, part, ',');	// -Y
		Rawdata.acc.Y = -stod(part) * convert_a * rate;
		getline(ss, part, ',');	// Y
		Rawdata.acc.X = stod(part) * convert_a * rate;

		// 读取陀螺仪数据
		getline(ss, part, ',');	// Z(弧度)
		Rawdata.gyr.Z = stod(part) * convert_g * rate;
		getline(ss, part, ',');	// -Y(弧度)
		Rawdata.gyr.Y = -stod(part) * convert_g * rate;
		getline(ss, part, '*');	// X(弧度)
		Rawdata.gyr.X = stod(part) * convert_g * rate;

		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}