//气动力系数计算文件
#pragma once
#include "threadpool.h"
#include "class.h"
#include "iterate.h"
R_z_theta R_theta_sa{0,0,0};//支杆和支撑机构姿态角
R_y_psi R_psi_sa{0,0,0};
R_x_phi R_phi_sa{0,0,0};

R_z_theta R_theta_ss{0,0,0};//支杆几何姿态角
R_y_psi R_psi_ss{0,0,0};
R_x_phi R_phi_ss{0,0,0};

MatrixXd LinearFit(MatrixXd &data)//线性拟合
{
    MatrixXd X = data.block(0,0,data.rows(),data.cols()-1);
    MatrixXd Y = data.block(0,data.cols()-1,data.rows(),1);

    return ((X.transpose()*X).inverse()*X.transpose()*Y);
}

VectorXd ElasticAngleCoef (vector<MatrixXd> &dataList,vector<MatrixXd> &loadHead710List,Map &coef,VectorXd &offset)//弹性角系数计算
{
    //????????????????????????头大了已经
    /*
    dataList:计算弹性角系数的数据，好几个文件得出。
    loadHead710List：加载头和710角度，由象限仪给出，计算滚转角时没有710角度。
    */
    //计算弹性角系数时，各个姿态角下的文件数量怎么考虑？
    //给的弹性角加载文件中，俯仰角文件36个，偏航26个，滚转14个(只用了7个，而且还是将第一个去除的)，总计76个。
    ThreadPool pool;//线程池
    vector<future<MatrixXd>> futures;//异步任务结果
    vector<MatrixXd> itResultList;//迭代结果列表
    VectorXd result;//弹性角系数，5个，顺序为KY,KMZ,KZ,KMY,KMX

    for(auto &data:dataList)
    {
        futures.push_back(pool.enqueue(ItResult, std::ref(data), std::ref(coef), std::ref(offset)));
    }
    for(auto &it:futures)
    {
        itResultList.push_back(it.get());
    }

    // MatrixXd mean_Y_Mz = MatrixXd::Zero(36,7);//带了因变量一列
    // mean_Y_Mz.col(6) = loadHead710List[0].col(0) - loadHead710List[0].col(1);
    // MatrixXd mean_Z_MY = MatrixXd::Zero(26,7);
    // mean_Z_MY.col(6) = loadHead710List[1].col(0) - loadHead710List[1].col(1);
    // MatrixXd mean_MX = MatrixXd::Zero(14,7);
    // mean_MX.col(6) = loadHead710List[2];//不用710角度

    // for(int i=0;i<76;i++)
    // {
    //     if(i<36){
    //         mean_Y_Mz.block(i,0,1,6) = itResultList[i].colwise().mean().transpose();
    //     }
    //     else if(i<62){
    //         mean_Z_MY.block(i-36,0,1,6) = itResultList[i].colwise().mean().transpose();
    //     }
    //     else{
    //         mean_MX.block(i-62,0,1,6) = itResultList[i].colwise().mean().transpose();
    //     }
    // }

    // result << LinearFit(mean_Y_Mz) , LinearFit(mean_Z_MY) , LinearFit(mean_MX) ;

    return result;
}

tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetInstallAngleTransMatrix(MatrixXd &InstallAngleData_0,
    MatrixXd &InstallAngleData_180,MatrixXd &InstallAngleData_90,MatrixXd &InstallAngleData_Neg90,Map &coef,VectorXd &offset)
{
    /*
    InstallAngleData:各角度下的安装角数据。
    offset: 修正值。
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