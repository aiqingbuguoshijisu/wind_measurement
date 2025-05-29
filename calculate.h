//气动力系数计算文件
#pragma once
#include "threadpool.h"
#include "class.h"
#include "iterate.h"
#include "tool.h"
#include <cmath>
enum COL {Y_COL,MZ_COL,MX_COL,X_COL,Z_COL,MY_COL};//只在选取迭代结果列时使用

//弹性角系数计算
VectorXd ElasticAngleCoef (vector<vector<MatrixXd>> &dataList,vector<MatrixXd> &loadHead710List,MatrixXd &coef,RowVectorXd &offset)//弹性角系数计算
{
    /*
    dataList:3个姿态角的数据文件的7元应变。
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

//安装角mb计算。double返回值是俯仰安装角mb，后面要用。
tuple<class R_z_theta, class R_y_psi, class R_x_phi, double> GetModelBalanceInstallAngleTransMatrix
(MatrixXd &InstallAngleData_0,MatrixXd &InstallAngleData_180,MatrixXd &InstallAngleData_90,
    MatrixXd &InstallAngleData_Neg90,MatrixXd &coef,RowVectorXd &offset)
{
    /*
    InstallAngleData:各角度下的安装角数据的7元应变。
    offset: 电压修正值。
    coef: 6*27个系数。
    */

    // 启动异步任务
    auto future_0 = async(launch::async, ItResult, ref(InstallAngleData_0), ref(coef), ref(offset));
    auto future_180 = async(launch::async, ItResult, ref(InstallAngleData_180), ref(coef), ref(offset));
    auto future_90 = async(launch::async, ItResult, ref(InstallAngleData_90), ref(coef), ref(offset));
    auto future_neg90 = async(launch::async, ItResult, ref(InstallAngleData_Neg90), ref(coef), ref(offset));

    // 获取结果（会自动等待线程完成）
    MatrixXd itResult_0 = future_0.get();
    MatrixXd itResult_180 = future_180.get();
    MatrixXd itResult_90 = future_90.get();
    MatrixXd itResult_Neg90 = future_neg90.get();

    double Theta_mb , Psi_mb, Phi_mb;

    double tmp1 = 0;
    double tmp2 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp1 += atan(itResult_0(i,X_COL)/itResult_0(i,Y_COL));
    }
    for(int i=0;i<itResult_180.rows();i++)
    {
        tmp2 += atan(itResult_180(i,X_COL)/itResult_180(i,Y_COL));
    }
    Theta_mb = Rad2Deg((tmp1/itResult_0.rows()+tmp2/itResult_180.rows())/2);//弧度制

    tmp1 = 0;
    tmp2 = 0;
    for(int i=0;i<itResult_90.rows();i++)
    {
        tmp1 += atan(itResult_90(i,X_COL)/itResult_90(i,Z_COL));
    }
    for(int i=0;i<itResult_Neg90.rows();i++)
    {
        tmp2 += atan(itResult_Neg90(i,X_COL)/itResult_Neg90(i,Z_COL));
    }
    Psi_mb = Rad2Deg((tmp1/itResult_90.rows()+tmp2/itResult_Neg90.rows())/(-2));

    tmp1 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp1 += atan(itResult_0(i,Z_COL)/itResult_0(i,Y_COL)); 
    }
    Phi_mb = Rad2Deg(tmp1/itResult_0.rows());

    R_z_theta R_theta_mb{Theta_mb,0,0};//支杆和支撑机构姿态角
    R_y_psi R_psi_mb{0,Psi_mb,0};
    R_x_phi R_phi_mb{0,0,Phi_mb};

    return make_tuple(R_theta_mb, R_psi_mb, R_phi_mb, Theta_mb);
}

//计算吹风弹性角bb，方法二。其中返回一个double是为了将初始弹性角弹出来后面要用的。VectorXd是计算气动力系数和自重修正时要用的载荷。
tuple<class R_z_theta, class R_y_psi, class R_x_phi, double, VectorXd> GetBalanceSupportRodElasticAngleTransMatrix
(VectorXd &ElasticAngleCoef,MatrixXd &data,MatrixXd &coef,RowVectorXd &offset,double SupportMechanismRollAngle)
{
    /*
    ElasticAngleCoef: 弹性角系数。
    data：吹风数据中的7元应变。
    coef: 6*27个系数。
    offset: 电压修正值。
    */
    //初始弹性角用0°按照数据计算。
    MatrixXd itResult = ItResult(data,coef,offset);
    RowVectorXd meanItResult = itResult.colwise().mean();
    double elasticThetaMethod1 = ElasticAngleCoef(0)*meanItResult(Y_COL) + ElasticAngleCoef(1)*meanItResult(MZ_COL);
    double elasticPsiMethod1 = ElasticAngleCoef(2)*meanItResult(Z_COL) + ElasticAngleCoef(3)*meanItResult(MY_COL);
    double elasticPhiMethod1 = ElasticAngleCoef(4)*meanItResult(MX_COL);

    //再计算第二部分的弹性角。
    double K_a = 2.88E-6 + (-5.74E-09)*fabs(SupportMechanismRollAngle) + 
        (2.33E-09)*pow(fabs(SupportMechanismRollAngle),2) + (-1.80E-11)*pow(fabs(SupportMechanismRollAngle),3);
    
    double K_b = 2.88E-6 + (-5.74E-09)*fabs(90-SupportMechanismRollAngle) + 
        (2.33E-09)*pow(fabs(90-SupportMechanismRollAngle),2) + (-1.80E-11)*pow(fabs(90-SupportMechanismRollAngle),3);

    double elasticThetaMethod2 = K_a * (2.2198 * meanItResult(Y_COL)+meanItResult(MZ_COL)); 
    double elasticPsiMethod2 = K_b * (2.2198 * meanItResult(Z_COL)+meanItResult(MY_COL));

    R_z_theta R_theta_bb{elasticThetaMethod1+elasticThetaMethod2,0,0};
    R_y_psi R_psi_bb{0,elasticPsiMethod1+elasticPsiMethod2,0};
    R_x_phi R_phi_bb{0,0,elasticPhiMethod1};

    return make_tuple(R_theta_bb, R_psi_bb, R_phi_bb,elasticThetaMethod1+elasticThetaMethod2,meanItResult.transpose());
}

// //计算支杆和支撑机构sa姿态角
// tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetSupportRodSupportingMechanismAngle(double Theta_sa,double Psi_sa,double Phi_sa)
// {
//     R_z_theta R_theta_sa{Theta_sa,0,0};//支杆和支撑机构姿态角
//     R_y_psi R_psi_sa{0,Psi_sa,0};
//     R_x_phi R_phi_sa{0,0,Phi_sa};
//     return make_tuple(R_theta_sa, R_psi_sa, R_phi_sa);
// }

// //计算支杆几何ss姿态角
// tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetSupportRodGeometricAngle(double Theta_ss,double Psi_ss,double Phi_ss)
// {
//     R_z_theta R_theta_ss{Theta_ss,0,0};//支杆几何姿态角
//     R_y_psi R_psi_ss{0,Psi_ss,0};
//     R_x_phi R_phi_ss{0,0,Phi_ss};
//     return make_tuple(R_theta_ss, R_psi_ss, R_phi_ss);
// }

R_z_theta R_theta_sa{0.0,0.0,0.0};//支杆和支撑机构姿态角
R_y_psi R_psi_sa{0.0,0.0,0.0};
R_x_phi R_phi_sa{0.0,0.0,0.0};

R_z_theta R_theta_ss{0.0,0.0,0.0};//支杆几何姿态角
R_y_psi R_psi_ss{0.0,0.0,0.0};
R_x_phi R_phi_ss{0.0,0.0,0.0};

//计算支撑机构ag姿态角
tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetSupportMechanismAngle
(double ScimitarPitchAngle,double ModelBalanceInstallPitchAngle,double InitialElasticAngle,double BalanceSupportRodPitchAngle
    ,double Phi_ag)
{
    /*
    ScimitarPitchAngle: 弯刀俯仰角。
    ModelBalanceInstallPitchAngle:模型天平俯仰安装角。
    InitialElasticAngle:初始弹性角。
    BalanceSupportRodPitchAngle：天平支杆俯仰安装角。暂时为0。
    Phi_ag: 支撑机构滚转角。
    */
    R_y_psi R_psi_ag{0,0,0};
    R_x_phi R_phi_ag{0,0,Phi_ag};
    //支撑机构俯仰角θag = 弯刀俯仰角 - 俯仰安装角mb - 初始弹性角 - 俯仰安装角bs（暂时0）
    double Theta_ag = ScimitarPitchAngle - ModelBalanceInstallPitchAngle - InitialElasticAngle - BalanceSupportRodPitchAngle;
    R_z_theta R_theta_ag{Theta_ag,0,0};
    return make_tuple(R_theta_ag, R_psi_ag, R_phi_ag);
}

//计算天平支杆安装角bs
tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetBalanceSupportInstallAngleTransMatrix
(double Theta_bs,double Psi_bs,double Phi_bs)
{
    R_z_theta R_theta_bs{Theta_bs,0,0};
    R_y_psi R_psi_bs{0,Psi_bs,0};
    R_x_phi R_phi_bs{0,0,Phi_bs};
    return make_tuple(R_theta_bs, R_psi_bs, R_phi_bs);
}

pair<double,double> CalculateAlphaBeta(Matrix3d &R_mg,double &delta_alpha,double &delta_beta)
{
    /*
    此函数表示从等式中计算α和β的过程。
    delta_alpha，delta_beta：角度制。
    */
    R_y_psi R_delta_beta{0,delta_beta,0};
    R_z_theta R_delta_alpha{delta_alpha,0,0};
    Vector3d vec{1,0,0};
    Vector3d vec_new = R_delta_beta.R_y_psi_N * R_delta_alpha.R_z_theta_N * vec;
    double alpha = atan2((R_mg.row(1).dot(vec_new)),(R_mg.row(0).dot(vec_new)));
    double beta = -asin(R_mg.row(2).dot(vec_new));

    alpha = Rad2Deg(alpha);
    beta = Rad2Deg(beta);
    return make_pair(alpha,beta);
}

VectorXd CalculateWindCoefN
(double &alpha,double &beta,VectorXd &F,Matrix3d &R_mb,double &p0,double &pct,double &delta_M,double &S)
{
    /*
    F：吹风时的载荷。这个参数是否用自重修正后的数据，有待商榷，因为自重的不确定度求解有点困难。
    alpha,beta：角度制。
    */
    //该函数的修正部分还未实现，后续再补充。
    Vector3d N_b {F(X_COL),F(Y_COL),F(Z_COL)};
    R_y_psi R_beta{0,-beta,0};
    R_z_theta R_alpha{-alpha,0,0};
    double tmp = pow(sqrt(5*(pow(pct/p0,-2/7)-1))+delta_M,2);
    double q = (0.7*p0*tmp)/pow((1+0.2*tmp),3.5);

    Vector3d CwN = (R_beta.R_y_psi_N * R_alpha.R_z_theta_N * R_mb *N_b)/(q*S);
    return CwN;
}

// pair<double,double> CalculateDeltaAlphaDeltaBeta
// (vector<Matrix3d> &R_mg,vector<VectorXd> &F,vector<Matrix3d> &R_mb,double &p0,double &pct,double &delta_M,double &S)
// {
//     /*
//     计算每个攻角下的数据
//     F：弹性角计算时会迭代出吹风时的载荷。
//     */
//     //这个函数用的0，180，+-90的数据独立于安装角的数据。暂时没给。
//     //先将Δα和Δβ设置为0。
//     double delta_alpha = 0;
//     double delta_beta = 0;
//     int count = R_mg.size();
//     MatrixXd X = MatrixXd::Zero(count,2);
//     X.col(1) = VectorXd::Ones(count);
//     VectorXd Y = VectorXd::Zero(count);
//     for(int i=0;i<count;i++){//0°安装时每个阶梯下的攻角
//         pair<double,double> tmp = CalculateAlphaBeta(R_mg[i],delta_alpha,delta_beta);
//         double alpha = tmp.first;
//         double beta = tmp.second;
//         VectorXd CwN = CalculateWindCoefN(alpha,beta,F[i],R_mb[i],p0,pct,delta_M,S);
//         if(alpha>=-2 && alpha<=2){
//             X(i,0) = alpha;
//             Y(i) = CwN(1);
//         }
//     }

//     VectorXd CoefIntercept = LinearFit(X,Y);
//     double K = CoefIntercept(0);//斜率
//     double alpha0positive = CoefIntercept(1);//0°安装截距

//     //选择实际攻角为0
    
// }