#include"IMU_Structs.h"

void Matrixprint(const MatrixXd Mat, const string name)
{
	cout << name << " (" << Mat.rows() << "x" << Mat.cols() << "):" << endl;
	IOFormat fmt(6, 0, " ", "\n", "", "", "", "");
	cout << Mat.format(fmt) << "\n" << "\n";
}

void Vectorprint(const VectorXd Mat, const string name)
{
	cout << name << " (" << Mat.rows() << "x" << Mat.cols() << "):" << endl;
	IOFormat fmt(6, 0, " ", "\n", "", "", "", "");
	cout << Mat.format(fmt) << "\n" << "\n";
}
//Matrixprint(V, "V");
//Matrixprint(W, "W");
//Matrixprint(Pos.Cbn, "Cbn");