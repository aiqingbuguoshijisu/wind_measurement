//声明Pose_Angle类
#pragma once
#include "tool.h"
//构造函数参数为弧度制。构造时输入角度制，自动转换成弧度。
class Pose_Angle//风轴系外的姿态角
{
    public:
        double THETA;
        double PSI;
        double PHI;
        Pose_Angle(double theta, double psi, double phi) : THETA(Deg2Rad(theta)), PSI(Deg2Rad(psi)), PHI(Deg2Rad(phi)) {}
        virtual void TransformMatrix()  = 0;
        virtual ~Pose_Angle() = default;  // 默认虚析构函数
};

class R_z_theta : public Pose_Angle//俯仰角转换矩阵
{
    public:
        R_z_theta(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_z_theta_N;
        Matrix3d R_z_theta_M;
        void TransformMatrix() override
        {
                R_z_theta_N << cos(THETA), -sin(THETA), 0,
                                sin(THETA),  cos(THETA), 0,
                                0,0,1;
                R_z_theta_M << cos(this->THETA), sin(this->THETA), 0,
                            -sin(this->THETA), cos(this->THETA), 0,
                            0,0,1;
        }
};

class R_y_psi : public Pose_Angle//偏航角转换矩阵
{
    public:
        R_y_psi(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_y_psi_N;
        Matrix3d R_y_psi_M;
        void TransformMatrix() override
        {
            R_y_psi_N << cos(this->PSI), 0, sin(this->PSI),
                        0, 1, 0,
                        -sin(this->PSI), 0, cos(this->PSI);
            R_y_psi_M << cos(this->PSI), 0, -sin(this->PSI),
                        0, 1, 0,
                        sin(this->PSI), 0, cos(this->PSI);
        }
};

class R_x_phi : public Pose_Angle//滚转角转换矩阵
{
    public:
        R_x_phi(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_x_phi_N;
        Matrix3d R_x_phi_M;
        void TransformMatrix() override
        {
            R_x_phi_N << 1, 0, 0,
                        0, cos(this->PHI), sin(this->PHI),
                        0, -sin(this->PHI), cos(this->PHI);
            R_x_phi_M << 1, 0, 0,
                        0, cos(this->PHI), sin(this->PHI),
                        0, -sin(this->PHI), cos(this->PHI);
        }
};