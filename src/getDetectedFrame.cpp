#include <iostream>
#include <Eigen/Dense>

int main()
{
    // 已知A的位姿
    Eigen::Vector3f translationA(1.0, 2.0, 3.0);  // A的平移部分
    Eigen::Quaternionf rotationA(0.7071, 0.0, 0.0, 0.7071);  // A的旋转部分（四元数表示）

    // 已知A到B的转移矩阵
    Eigen::Matrix4f transformationMatrixAB;
    // 假设已知的转移矩阵为：
    // transformationMatrixAB << ...;

    // 分解转移矩阵
    Eigen::Matrix3f rotationMatrixB = transformationMatrixAB.block<3, 3>(0, 0);
    Eigen::Vector3f translationB = transformationMatrixAB.block<3, 1>(0, 3);

    // 提取旋转和平移信息
    Eigen::Quaternionf rotationB(rotationMatrixB);

    // 求解B的位姿
    Eigen::Vector3f translationResult = translationA + rotationA * translationB;
    Eigen::Quaternionf rotationResult = rotationA * rotationB;

    // 打印B的位姿
    std::cout << "Translation of B: " << translationResult.transpose() << std::endl;
    std::cout << "Rotation of B (Quaternion): " << rotationResult.coeffs().transpose() << std::endl;

    return 0;
}
