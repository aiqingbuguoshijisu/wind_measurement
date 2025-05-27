//气动力系数计算文件
#pragma once
#include "threadpool.h"
#include "class.h"
#include "iterate.h"
#include "tool.h"
enum COL {Y_COL,MZ_COL,MX_COL,X_COL,Z_COL,MY_COL};
R_z_theta R_theta_sa{0,0,0};//支杆和支撑机构姿态角
R_y_psi R_psi_sa{0,0,0};
R_x_phi R_phi_sa{0,0,0};

R_z_theta R_theta_ss{0,0,0};//支杆几何姿态角
R_y_psi R_psi_ss{0,0,0};
R_x_phi R_phi_ss{0,0,0};

VectorXd LinearFit(MatrixXd &X,VectorXd &Y)//线性拟合
{
    //return ((X.transpose()*X).inverse()*X.transpose()*Y);
    return X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
}

VectorXd ElasticAngleCoef (vector<vector<MatrixXd>> &dataList,vector<MatrixXd> &loadHead710List,MatrixXd &coef,RowVectorXd &offset)//弹性角系数计算
{
    /*
    dataList:3个姿态角的数据文件。
    loadHead710List：加载头和710角度，由象限仪给出，计算滚转角时没有710角度。
    coef: 6*27个系数。
    offset: 电压修正值。
    */
   /*
   在俯仰弹性角文件里面添加了“250224俯仰前点0块回-1”，“250225俯仰后点0块回-1”文件。
   在偏航弹性角里面添加了“250225偏航前点1块回-1”，“250225偏航后点1块回-1”文件。
   形成完整的小循环。
   */

    ThreadPool pool;//线程池
    vector<VectorXd> allResult;//放三组弹性角系数
    VectorXd result = VectorXd::Zero(5);//弹性角系数，5个，顺序为KY,KMZ,KZ,KMY,KMX
    int processIndex = 0;//前两个弹性角系数计算类似，滚转角的弹性角系数计算单独写。
    for(auto &data:dataList)
    {
        vector<future<MatrixXd>> tmp_futures;//异步任务结果
        vector<MatrixXd> tmp_itResult;//迭代结果列表

        for(auto &it:data)
        {
            tmp_futures.push_back(pool.enqueue(ItResult, std::ref(it), std::ref(coef), std::ref(offset)));
        }
        for(auto &it:tmp_futures)
        {
            tmp_itResult.push_back(it.get());
        }
        int tmp_itResult_size = tmp_itResult.size();
        //itResultList.push_back(tmp_itResult);
        if(processIndex < 2){
            MatrixXd meanMatrix=MatrixXd::Zero(tmp_itResult_size,6);
            int fixIndex = tmp_itResult_size/4;
            RowVectorXd tmp = RowVectorXd::Zero(6);
            for(int i=0;i<tmp_itResult_size;i++){
                if(i%fixIndex == 0){
                    tmp = tmp_itResult[i].colwise().mean();
                }
                meanMatrix.row(i) = tmp_itResult[i].colwise().mean() - tmp;
            }
            VectorXd Y = loadHead710List[processIndex].col(0) - loadHead710List[processIndex].col(1);
            MatrixXd X = MatrixXd::Zero(tmp_itResult_size,2);
            if(processIndex == 0) X << meanMatrix.col(Y_COL), meanMatrix.col(MZ_COL);
            else X << meanMatrix.col(Z_COL), meanMatrix.col(MY_COL);
            allResult.push_back(LinearFit(X,Y));
            processIndex++;
        }
        else{
            MatrixXd meanMatrix=MatrixXd::Zero(tmp_itResult_size,6);  
            int fixIndex1 = tmp_itResult_size/2;
            //int fixIndex1 = (tmp_itResult_size-2)/4;//滚转角暂时还没给全，目前就是只有7个数据文件
            //int fixIndex2 = 3*fixIndex1+1;
            RowVectorXd tmp = RowVectorXd::Zero(6);
            for(int i=0;i<tmp_itResult_size;i++){
                // if(i<tmp_itResult_size/2){
                //     tmp = tmp_itResult[fixIndex1].colwise().mean();
                // }
                // else{
                //     tmp = tmp_itResult[fixIndex2].colwise().mean();
                // }
                tmp = tmp_itResult[fixIndex1].colwise().mean();
                meanMatrix.row(i) = tmp_itResult[i].colwise().mean() - tmp;
            }
            VectorXd Y = loadHead710List[processIndex].col(0);
            MatrixXd X = MatrixXd::Zero(tmp_itResult_size,1);
            X<<meanMatrix.col(MX_COL);
            allResult.push_back(LinearFit(X,Y));
        }
    }
    result.segment(0,allResult[0].size()) = allResult[0];
    result.segment(allResult[0].size(),allResult[1].size()) = allResult[1];
    result.segment(allResult[0].size()+allResult[1].size(),allResult[2].size()) = allResult[2];

    return result;
}

tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetInstallAngleTransMatrix(MatrixXd &InstallAngleData_0,
    MatrixXd &InstallAngleData_180,MatrixXd &InstallAngleData_90,MatrixXd &InstallAngleData_Neg90,MatrixXd &coef,RowVectorXd &offset)
{
    /*
    InstallAngleData:各角度下的安装角数据。
    offset: 电压修正值。
    coef: 6*27个系数。
    */

    // MatrixXd itResult_0 = ItResult(InstallAngleData_0,offset,coef);
    // MatrixXd itResult_180 = ItResult(InstallAngleData_180,offset,coef);
    // MatrixXd itResult_90 = ItResult(InstallAngleData_90,offset,coef);
    // MatrixXd itResult_Neg90 = ItResult(InstallAngleData_Neg90,offset,coef);

    // 启动异步任务
    auto future_0 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_0), std::ref(coef), std::ref(offset));
    auto future_180 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_180), std::ref(coef), std::ref(offset));
    auto future_90 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_90), std::ref(coef), std::ref(offset));
    auto future_neg90 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_Neg90), std::ref(coef), std::ref(offset));

    // 获取结果（会自动等待线程完成）
    MatrixXd itResult_0 = future_0.get();
    MatrixXd itResult_180 = future_180.get();
    MatrixXd itResult_90 = future_90.get();
    MatrixXd itResult_Neg90 = future_neg90.get();

    double Theta_mb , Psi_mb, Phi_mb;
    double tmp1 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp1 += atan(itResult_0(i,X_COL)/itResult_0(i,Y_COL));
    }
    double tmp2 = 0;
    for(int i=0;i<itResult_180.rows();i++)
    {
        tmp2 += atan(itResult_180(i,X_COL)/itResult_180(i,Y_COL));
    }
    double tmp3 = 0;
    for(int i=0;i<itResult_90.rows();i++)
    {
        tmp3 += atan(itResult_90(i,X_COL)/itResult_90(i,Z_COL));
    }
    double tmp4 = 0;
    for(int i=0;i<itResult_Neg90.rows();i++)
    {
        tmp4 += atan(itResult_Neg90(i,X_COL)/itResult_Neg90(i,Z_COL));
    }
    double tmp5 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp5 += atan(itResult_0(i,Z_COL)/itResult_0(i,Y_COL)); 
    }

    Theta_mb = Rad2Deg((tmp1/itResult_0.rows()+tmp2/itResult_180.rows())/2);//弧度制
    Psi_mb = Rad2Deg((tmp3/itResult_90.rows()+tmp4/itResult_Neg90.rows())/(-2));
    Phi_mb = Rad2Deg(tmp5/itResult_0.rows());

    R_z_theta R_theta_mb{Theta_mb,0,0};//支杆和支撑机构姿态角
    R_y_psi R_psi_mb{0,Psi_mb,0};
    R_x_phi R_phi_mb{0,0,Phi_mb};

    return make_tuple(R_theta_mb, R_psi_mb, R_phi_mb);
}