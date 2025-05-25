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
#define Map map<int, vector<double>>

enum COL {Y_COL,MZ_COL,MX_COL,X_COL,Z_COL,MY_COL};

void printResult(const map<int, std::vector<double>>& result) {
    // 打印第一个元素：map<int, vector<double>>
    std::cout << "First element (map<int, vector<double>>):\n";
    for (const auto& elem : result) {
        std::cout << "Key: " << elem.first << ", Values: ";
        for (double val : elem.second) {
            std::cout <<fixed<<setprecision(2)<< val << " ";
        }
        std::cout << "\n";
    }
}

MatrixXd MapToMat(Map &data)//将map<int ,vector<double>>数据转化为MatrixXd数据
{
    int rowCount = data.size();
    int colCount = data.begin()->second.size();
    MatrixXd result(rowCount, colCount);
    int index = 0;
    for (const auto& pair : data) 
    {
        for (int j = 0; j < colCount; j++) 
        {
            result(index, j) = pair.second[j];
        }
        index++;
    }
    return result;
}

Map MatToMap(MatrixXd &m)
{
    Map result;
    for(int i = 0;  i < m.rows(); ++i)
    {
        vector<double> tmp(m.cols());
        for(int j =0;j<m.cols();++j)
        {
            tmp[j] = m(i,j);
        }
        result.emplace(i, std::move(tmp));
    }
    return result;
}

double Rad2Deg(double rad)
{
    return rad*180/M_PI;
}
double Deg2Rad(double deg)
{
    return deg*M_PI/180;
}