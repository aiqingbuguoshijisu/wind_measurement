#pragma once
/*
* 1.函数的命名第一个字母大写，中间不加符号。
* 2.变量名中，如果出现-90°这种数据，使用Neg90命名。
* 3.计算出角度后需转换成角度制。
* 4.函数的输入参数尽量不要是文件路径。
*/
//C++17及以上版本运行 

#include <iostream>
#include <vector>
#include <map>
#include<iomanip>
#include "../eigen-3.4.0/Eigen/Dense"
#include "../eigen-3.4.0/Eigen/Core"
using namespace Eigen;
using namespace std;
#define M_PI 3.14159265358979323846

double Rad2Deg(double rad)//将弧度转换为角度制
{
    return rad*180/M_PI;
}

double Deg2Rad(double deg)//将角度转换为弧度制
{
    return deg*M_PI/180;
}


VectorXd LinearFit(MatrixXd &X,VectorXd &Y)//线性拟合
{
    //return ((X.transpose()*X).inverse()*X.transpose()*Y);
    return X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
}