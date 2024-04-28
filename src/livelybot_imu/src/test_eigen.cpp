#include <iostream>
#include <Eigen/Dense>


class MyClass {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyClass(){
        Eigen::Quaterniond q_z=  
      Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ())
      *
      Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY())
      *
      Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitX());
    }
private:
    Eigen::Vector4d myVector;
};

int main() {
    // 初始化Eigen库中的四元数，表示绕Z轴旋转180度
    // const double pi = Eigen::numbers::pi<double>(); // 使用Eigen的pi常量
      Eigen::Quaterniond q_z=  
      Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ())
      *
      Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY())
      *
      Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitX());
    // 输出四元数的各个分量
    std::cout << "Quaternion components:\n";
    std::cout << "w: " << q_z.w() << "\n";
    std::cout << "x: " << q_z.x() << "\n";
    std::cout << "y: " << q_z.y() << "\n";
    std::cout << "z: " << q_z.z() << "\n\n";

    // 转换为旋转矩阵并输出
    Eigen::Matrix3d rotation_matrix = q_z.toRotationMatrix();
    std::cout << "Rotation matrix:\n" << rotation_matrix << std::endl;

    return 0;
}