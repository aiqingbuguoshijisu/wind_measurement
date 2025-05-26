//迭代计算文件
#pragma once
#include "tool.h"

const int it_nums = 7;
// map<int, vector<double>> CombinedVariables (vector<double> &Y)//计算干扰项的自变量，按顺序放
// {
//     /*
//     Y：载荷值。
//     */
//     map<int, vector<double>> m1;//一阶项
//     for(int i=0;i<6;i++)
//     {
//         m1[i] = vector<double>{Y[0], Y[1], Y[2], Y[3], Y[4], Y[5]};
//         m1[i].erase(m1[i].begin()+i);//剔除对角线上的自变量
//     }

//     vector<double> m2;//二阶项
//     for(int i=0;i<6;i++)
//     {
//         m2.push_back(Y[i]*Y[i]);
//     }
//     for(int j=0;j<5;j++)
//     {
//         for(int k=j+1;k<6;k++)
//         {
//             m2.push_back(Y[j]*Y[k]);
//         }
//     }
//     for(auto& it:m1)
//     {
//         it.second.insert(it.second.end(),m2.begin(),m2.end());
//     }

//     return m1;//返回自变量矩阵
// }

// map<int, vector<double>> IterativeCalculation(vector<double> &U,map<int, vector<double>> &coef)
// {
//     /*
//     coef：6*27个系数
//     U：各载荷下产生的电压，修正组桥后的
//     */
//     //计算主项力Y_0
//     vector <double> Y_0(6,0);
//     for(int i=0;i<6;i++)
//     {
//         //Y_0[i] = coff[i][i]*U[i]/1000;
//         Y_0[i] = coef[i][i]*U[i]/1000;
//     }
//     //map<int, vector<double>> tmp_coff = coff;
//     map<int, vector<double>> tmp_coff = coef;
//     int index = 0;//剔除对角线上的系数，只保留干扰项的系数
//     for (auto& it:tmp_coff)
//     {
//         it.second.erase(it.second.begin()+index);
//         index++;
//     }
//     //迭代过程
//     map<int, vector<double>> Y_i;//存放迭代结果

//     vector <double> sum(6,0);//每个载荷的干扰项值
//     map<int, vector<double>> x = CombinedVariables(Y_0);//利用Y_0计算出自变量矩阵
//     vector<double> tmp;
//     for (int i=0;i<6;i++)
//     {
//         for (int j=0;j<tmp_coff[i].size();j++)
//         {
//             sum[i] += x[i][j] * tmp_coff[i][j] ;//将自变量与对应的系数相乘得到各载荷的干扰项值
//         }
//         tmp.push_back(Y_0[i] + sum[i]);
//     }
//     Y_i[0] = tmp;//第一次迭代结果

//     for(int i=0;i<(it_nums-1);i++)//上面已经迭代一次了，这里再迭代6次
//     {
//         vector<double> sum1(6,0);
//         map<int, vector<double>> x = CombinedVariables(Y_i[i]);//使用上一次的载荷计算下一次的自变量矩阵
//         vector<double> tmp1;
//         for (int j=0;j<6;j++)
//         {
//             for (int k=0;k<tmp_coff[j].size();k++)
//             {
//                 sum1[j] += x[j][k] * tmp_coff[j][k] ;
//             }
//             tmp1.push_back(Y_0[j] + sum1[j]);
//         }
//         Y_i[i+1] = tmp1;
//     }

//     return Y_i;//返回迭代结果
// }
MatrixXd CombinedVariables(const RowVectorXd& Y) // 计算干扰项的自变量，按顺序放
{
    /*
    Y：载荷值。
    */
    
    // First order terms - create 6x5 matrix (excluding diagonal elements)
    MatrixXd m1(6, 5);
    for(int i = 0; i < 6; i++) {
        int col_idx = 0;
        for(int j = 0; j < 6; j++) {
            if(j != i) { // 剔除对角线上的自变量
                m1(i, col_idx) = Y(j);
                col_idx++;
            }
        }
    }

    // Second order terms
    RowVectorXd m2(21); // 6 quadratic + 15 cross terms = 21 terms
    int idx = 0;
    
    // Quadratic terms
    for(int i = 0; i < 6; i++) {
        m2(idx++) = Y(i) * Y(i);
    }
    
    // Cross terms
    for(int j = 0; j < 5; j++) {
        for(int k = j + 1; k < 6; k++) {
            m2(idx++) = Y(j) * Y(k);
        }
    }

    // Combine first and second order terms
    MatrixXd result(6, 26); // 5 first order + 21 second order = 26 columns
    result.block(0, 0, 6, 5) = m1;
    for(int i = 0; i < 6; i++) {
        result.block(i, 5, 1, 21) = m2;
    }

    return result; // 返回自变量矩阵
}

RowVectorXd IterativeCalculation(const RowVectorXd& U, const MatrixXd& coef)
{
    /*
    coef：6*27个系数
    U：各载荷下产生的电压，修正组桥后的
    */
    
    // 计算主项力Y_0
    RowVectorXd Y_0(6);
    for(int i = 0; i < 6; i++) {
        Y_0(i) = coef(i, i) * U(i) / 1000.0;
    }
    
    // Create coefficient matrix without diagonal elements (6x26)
    MatrixXd tmp_coef(6, 26);
    for(int i = 0; i < 6; i++) {
        int col_idx = 0;
        for(int j = 0; j < 27; j++) { // Assuming coef is 6x27
            if(j != i) { // 剔除对角线上的系数，只保留干扰项的系数
                tmp_coef(i, col_idx) = coef(i, j);
                col_idx++;
            }
        }
    }
    
    // 迭代过程
    //MatrixXd Y_i(it_nums, 6);
    RowVectorXd Y_current = Y_0; // 当前迭代结果
    // First iteration  
    RowVectorXd sum = RowVectorXd::Zero(6); // 每个载荷的干扰项值
    MatrixXd x = CombinedVariables(Y_0); // 利用Y_0计算出自变量矩阵
    
    for(int i = 0; i < 6; i++) {
        sum(i) = tmp_coef.row(i).dot(x.row(i)); // 将自变量与对应的系数相乘得到各载荷的干扰项值
    }
    
    // Y_i.row(0) = Y_0 + sum; // 第一次迭代结果
    
    // // Subsequent iterations
    // for(int iter = 1; iter < it_nums; iter++) { // 上面已经迭代一次了，这里再迭代剩余次数
    //     RowVectorXd sum1 = RowVectorXd::Zero(6);
    //     MatrixXd x1 = CombinedVariables(Y_i.row(iter-1)); // 使用上一次的载荷计算下一次的自变量矩阵
        
    //     for(int j = 0; j < 6; j++) {
    //         sum1(j) = tmp_coef.row(j).dot(x1.row(j));
    //     }
        
    //     Y_i.row(iter) = Y_0 + sum1;
    // }
    // Iterative process
    for(int iter = 0; iter < it_nums; iter++) {
        RowVectorXd sum = RowVectorXd::Zero(6); // 每个载荷的干扰项值
        MatrixXd x = CombinedVariables(Y_current); // 利用当前载荷计算自变量矩阵
        
        for(int i = 0; i < 6; i++) {
            sum(i) = tmp_coef.row(i).dot(x.row(i)); // 将自变量与对应的系数相乘得到各载荷的干扰项值
        }
        
        Y_current = Y_0 + sum; // 更新迭代结果
    }
    return Y_current; // 返回迭代结果
}
MatrixXd BridgeCorrect(const MatrixXd &rawVol,const RowVectorXd &offset)//先修正，后组桥
{
    /*
    rawVol: 原始7元应变。
    offset: 电压修正值。
    */
    //修正电压值怎么给，每次都给在Balance Parameters (17-N6-80A)20250228.dat里面吗？
    MatrixXd correctedVol = rawVol.rowwise() - offset;
    // 13.11727924	,110.0940311	,93.70432627	,-79.92551249	,448.9582912	,-197.00641     ,-27.41440414
    MatrixXd result(rawVol.rows(), 6);
    result.col(0) = correctedVol.col(1) - correctedVol.col(0);//uy
    result.col(1) = correctedVol.col(1) + correctedVol.col(0);//umz
    result.col(2) = correctedVol.col(2);//umx
    result.col(3) = correctedVol.col(3) + correctedVol.col(6);//ux
    result.col(4) = correctedVol.col(4) - correctedVol.col(5);//uz
    result.col(5) = correctedVol.col(4) + correctedVol.col(5);//umy

    return result;
}

MatrixXd ItResult(const MatrixXd &data,const MatrixXd &coef,const RowVectorXd &offset)//迭代结果
{
    /*
    data : 读取的原始数据，函数里面有选取7元应变的过程。
    offset ：电压修正值。
    coef ：6*27个系数。
    */
    MatrixXd result_seven = MatrixXd::Zero(data.rows(), 6);
    MatrixXd datablock = data.block(0,24,data.rows(),7);
    MatrixXd datablockFix = BridgeCorrect(datablock,offset);
   // Map data_map = MatToMap(datablockFix);

   for(int i=0;i<datablockFix.rows();i++){
        result_seven.row(i) = IterativeCalculation(datablockFix.row(i), coef);
   }

    return result_seven;
}

// map<int ,vector<double>> _Result(string dataFilePath,string coffFilePath)//测试用
// {
//     map<int, vector<double>> result_seven;

//     map<int, vector<double>> coef = ReadFactorFile(coffFilePath);
//     MatrixXd data = ReadNormalData(dataFilePath,4);

//     MatrixXd datablock = data.block(0,24,data.rows(),7);
//     MatrixXd datablockFix = BridgeCorrect(datablock);

//     Map data_map = MatToMap(datablockFix);
//     for(auto& it:data_map)
//     {
//         map<int, vector<double>> Y_i = IterativeCalculation(it.second);
//         result_seven[it.first] = Y_i.rbegin()->second;
//     }
//     return result_seven;
// }