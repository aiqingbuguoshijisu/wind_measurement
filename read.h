//文件读取文件
#pragma once
#include "tool.h"
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
// 读取常规的数据文件，如安装角，原始数据等。并且删除一些没用的行。
MatrixXd ReadNormalData(const string &filename ,int DeleteLinesCounts)
{
    std::filesystem::path fsPath;
    try {
        // Use u8path to interpret the input string as UTF-8
        fsPath = std::filesystem::u8path(filename);
    } catch (const std::exception& e) {
        cerr << "Error: Invalid file path format: " << filename << " - " << e.what() << endl;
        return {};
    }
    ifstream inFile(fsPath);
    if (!inFile.is_open()) {
        cerr << "Error: Unable to open file " << fsPath << endl;
        return {};
    }

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

    if (lines.empty()) {
    cerr << "Error: File is empty" << endl;
    return {};
    }
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
    if (cols == 0) {
        cerr << "Error: First data line contains no numeric values." << endl;
        return MatrixXd();
    }

    // 读取数据
    MatrixXd data(lines.size(), cols);//不带表头的数据大小
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
            data(row, col++) = val;
        }
        ++row;
    }
    
    return data;
}

//弹性角系数计算要读取整个文件夹的数据
vector<MatrixXd> ReadFolderAllData(const string &folderPath)
{
    int deleteLinesCounts = 4;
    vector<MatrixXd> dataList;
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
        dataList.push_back(ReadNormalData(path.string(), deleteLinesCounts));
    }
    
    return dataList;
}

// map<int ,vector<double>> ReadFactorFile(const string& coffFilePath)//读系数文件，并转换成6*27，方便后续使用
// {
//     std::filesystem::path fsPath;
//     try {
//          // Use u8path to interpret the input string as UTF-8
//         fsPath = std::filesystem::u8path(coffFilePath);
//     } catch (const std::exception& e) {
//         cerr << "Error: Invalid file path format: " << coffFilePath << " - " << e.what() << endl;
//             return map<int, vector<double>>();
//     }
//     ifstream inFile(fsPath);
//     if (!inFile.is_open()) {
//         cerr << "Error: Unable to open file " << fsPath << endl;
//         return map<int, vector<double>>();
//     }

//     vector<string> lines;
//     string line;
//     int rowCount = 0;
//     while (getline(inFile, line) && rowCount < 28) {//拿28行数据，但是第一行是表头，后面会删除。
//         if(line.find_first_not_of(" \t") == string::npos)
//         {
//             continue;
//         }
//         lines.push_back(line);
//         ++rowCount;
//     }
//     inFile.close();

//     if (lines.empty()) {
//         cerr << "Error: File is empty" << endl;
//         return map<int, vector<double>>();
//     }

//     //如果第一行是标题行，可以删除（根据实际情况决定）
//     lines.erase(lines.begin());

//     map<int, vector<double>> data;
//     int dataIndex = 0;

//     for (int i = 0; i < lines.size(); ++i) {
//         stringstream ss(lines[i]);
//         vector<double> row;
//         double value;
//         while (ss >> value) {
//             row.push_back(value);
//         }
//         data[dataIndex] = row;
//         dataIndex++;
//     }

//     map<int, vector<double>> data_T;
//     for(int j =0;j<6;j++)
//     {
//         vector<double> row;
//         for(int i=0;i<data.size();i++)
//         {
//             row.push_back(data[i][j]);
//         }
//         data_T[j] = row;
//     }

//     return data_T;
// }

MatrixXd ReadFactorFile(const string& coffFilePath) // 读系数文件，并转换成6*27，方便后续使用
{
    std::filesystem::path fsPath;
    try {
        // Use u8path to interpret the input string as UTF-8
        fsPath = std::filesystem::u8path(coffFilePath);
    } catch (const std::exception& e) {
        cerr << "Error: Invalid file path format: " << coffFilePath << " - " << e.what() << endl;
        return MatrixXd();
    }
    
    ifstream inFile(fsPath);
    if (!inFile.is_open()) {
        cerr << "Error: Unable to open file " << fsPath << endl;
        return MatrixXd();
    }

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

    if (lines.empty()) {
        cerr << "Error: File is empty" << endl;
        return MatrixXd();
    }

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

    if (tempData.empty()) {
        cerr << "Error: No valid data found" << endl;
        return MatrixXd();
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

