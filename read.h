//文件读取文件
#pragma once
#include "threadpool.h"
#include <iostream>
#include <vector>
#include<iomanip>
#include "../eigen-3.4.0/Eigen/Dense"
#include "../eigen-3.4.0/Eigen/Core"
#include <filesystem>
#include <fstream>
using namespace Eigen;
using namespace std;

namespace fs = std::filesystem;
// 读取常规的数据文件中除时间戳以外的全部数据，此外还将应变单独复制一份返回，如安装角，原始数据等。并且删除一些没用的行。
pair<MatrixXd,MatrixXd> ReadNormalData(const string &filename ,int DeleteLinesCounts)
{
    std::filesystem::path fsPath;
    fsPath = std::filesystem::u8path(filename);
    ifstream inFile(fsPath);

    vector<string> lines;
    string line;
    while (getline(inFile, line)) { 
        if(line.find_first_not_of(" \t") == string::npos)
        {
            continue;
        }
        lines.push_back(line);
    }
    inFile.close();

    //删除前四行包括表头
    lines.erase(lines.begin(), lines.begin() + DeleteLinesCounts);
    
    // 获取列数
    int cols = 0;
    stringstream ss(lines[0]);
    string time1, time2;
    ss >> time1 >> time2;// 跳过两个时间戳
    double val;
    while (ss >> val) {
        ++cols;
    }

    // 读取数据
    MatrixXd alldata(lines.size(), cols);//不带表头的数据大小
    int row = 0;
    for(const auto& line : lines)
    {
        stringstream ss(line);
        string time;

        for(int i = 0; i < 2; ++i)
        {
            if (!(ss >> time)) break; // 跳过两个时间戳
        }

        int col = 0;
        double val;
        while(ss>>val && col < cols)
        {
            alldata(row, col++) = val;
        }
        ++row;
    }
    MatrixXd straindata = alldata.block(0,24,alldata.rows(),7);
    return make_pair(alldata,straindata);
}

//弹性角系数计算要读取整个文件夹的数据
pair<vector<MatrixXd>,vector<MatrixXd>> ReadFolderAllData(const string &folderPath)
{
    int deleteLinesCounts = 4;
    ThreadPool pool;
    vector<future<pair<MatrixXd,MatrixXd>>> futures;
    vector<MatrixXd> alldataList;
    vector<MatrixXd> straindataList;
    vector<std::pair<fs::path, fs::file_time_type>> files;// 用于存储文件路径及其最后修改时间

    for(const auto& entry : fs::directory_iterator(folderPath)){
        if (entry.is_regular_file()) {
            files.emplace_back(entry.path(), entry.last_write_time());
        }
    }

    // 按照修改时间升序排序
    sort(files.begin(), files.end(),[](const auto& a, const auto& b) {
        return a.second < b.second;  // 升序排列
    });

    for(const auto& [path,_] :files){
        futures.push_back(pool.enqueue(ReadNormalData,path.string(),deleteLinesCounts));
    }
    for(auto &f : futures){
        auto result = f.get(); // 仅调用一次 get()
        alldataList.push_back(result.first);
        straindataList.push_back(result.second);
    }
    return make_pair(alldataList,straindataList);
}

MatrixXd ReadFactorFile(const string& coefFilePath) // 读系数文件，并转换成6*27，方便后续使用
{
    std::filesystem::path fsPath;
    fsPath = std::filesystem::u8path(coefFilePath);
    
    ifstream inFile(fsPath);

    vector<string> lines;
    string line;
    int rowCount = 0;
    while (getline(inFile, line) && rowCount < 28) { // 拿28行数据，但是第一行是表头，后面会删除。
        if(line.find_first_not_of(" \t") == string::npos) {
            continue;
        }
        lines.push_back(line);
        ++rowCount;
    }
    inFile.close();

    // 如果第一行是标题行，可以删除（根据实际情况决定）
    lines.erase(lines.begin());

    // Parse data into temporary vector structure
    vector<vector<double>> tempData;
    for (const auto& line : lines) {
        stringstream ss(line);
        vector<double> row;
        double value;
        while (ss >> value) {
            row.push_back(value);
        }
        if (!row.empty()) {
            tempData.push_back(row);
        }
    }

    // Convert to Eigen matrix and transpose (6 rows, 27 columns)
    int rows = tempData.size();
    int cols = tempData[0].size();
    
    MatrixXd data(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            data(i, j) = tempData[i][j];
        }
    }

    // Transpose to get 6x27 matrix (assuming we want first 6 columns transposed)
    MatrixXd data_T = data.transpose();
    
    return data_T;
}

MatrixXd ReadLoad710AngleData(const string& load710FilePath)
{
    std::filesystem::path fsPath;
    fsPath = std::filesystem::u8path(load710FilePath);
    
    ifstream inFile(fsPath);

    vector<string> lines;
    string line;
    while (getline(inFile, line)) {
        if(line.find_first_not_of(" \t") == string::npos) {
            continue;
        }
        lines.push_back(line);
    }
    inFile.close();
    lines.erase(lines.begin());
    vector<vector<double>> tempData;
    for (const auto& line : lines) {
        stringstream ss(line);
        vector<double> row;
        double value;
        while (ss >> value) {
            row.push_back(value);
        }
        if (!row.empty()) {
            tempData.push_back(row);
        }
    }

    int rows = tempData.size();
    int cols = tempData[0].size();
    
    MatrixXd data(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            data(i, j) = tempData[i][j];
        }
    }
    return data;
}