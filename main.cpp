#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <armadillo>

using namespace std;

int main() {
    //给定旋转向量，小量
    Eigen::Vector3d w(0.01, 0.02, 0.03);
    //设置一个旋转向量，转化为旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    //获得相应的四元数
    Eigen::Quaterniond q(R);
//现在考虑在旋转向量以及较小的扰动下的更新
//数据输出


    cout << "旋转之前的旋转矩阵是：" << endl;
    cout << R << endl;
    cout << "旋转之前的四元数是：" << endl;
    cout << q.coeffs().transpose() << endl;

    //更新方式一，使用罗德里格公式进行更新
    double theta = w.norm();//旋转向量对应的旋转角度归一化处理
    Eigen::Vector3d n_w = w / theta;//旋转向量对应的旋转轴
    Eigen::Matrix3d n_w_skew;//输入旋转向量对应的^反对称矩阵
    n_w_skew << 0, -n_w(2), n_w(1),
            n_w(2), 0, -n_w(0),
            -n_w(1), n_w(0), 0;
    //罗德里格公式表示的旋转
    Eigen::Matrix3d R_w =
            cos(theta) * Eigen::Matrix3d::Identity() + (1 - cos(theta)) * n_w * n_w.transpose() + sin(theta) * n_w_skew;
    //进行更新
    Eigen::Matrix3d R_update = R * R_w;
    cout << "旋转矩阵的更新是：" << endl;
    cout << R_update << endl;

    //四元数进行更新，求导数之后的结果
    Eigen::Quaterniond q_w(1, w(0) / 2, w(1) / 2, w(2) / 2);
    //进行更新
    Eigen::Quaterniond q_update = q * q_w;
    //归一化，只有归一化之后才表示一个旋转
    q_update = q_update.normalized();
    cout << "四元数表示的更新是：" << endl;
    cout << q_update.coeffs().transpose() << endl;
    cout << "二者的相对误差用矩阵表示是：" << endl;
    cout << q_update.toRotationMatrix() - R_update << endl;
    cout << "二者的相对误差四元数表示是：" << endl;
    Eigen::Quaterniond q1(R_update);
    cout << q_update.coeffs().transpose() - q1.coeffs().transpose() << endl;


    std::cout << "Hello,wpf!" << std::endl;


    return 0;
}

