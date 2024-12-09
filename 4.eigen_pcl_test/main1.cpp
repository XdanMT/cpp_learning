// 旋转矩阵获取方式，并解释了该矩阵如何使用

#include <iostream>
#include <string>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>


int main(){
	// 初始化点的
	Eigen::Vector3f orin_point(3.0, 4.0, 0.0);
	Eigen::Vector3f orin_point2(5.0, 0.0, 0.0);


	// 找到该点的方向 theta 度，（将原始参考坐标系，绕z轴旋转到目标坐标系后使得xoy平面重合，经过的角度）
	float start_orientation = atan2(orin_point(1), orin_point(0)); 

    
	// 构造一个旋转操作对应的数据结构：原始参考坐标系绕Z轴旋转 theta 角
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ()); 
	// 返回的是一个3×3的矩阵，表示 旋转前->旋转后 的直乘矩阵 
    Eigen::Matrix3f rotate_matrix = t_V.matrix();                       
    // Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();     // 创建4×4的单位矩阵

	// 点的旋转方式1：将原始点（参考系下的）旋转 theta 角之后的坐标，结果为 (-1.4, 4.8, 0)
	//			    或者说固定点不动，将原始参考坐标系旋转 -theta 角之后，点的新坐标表示。
	// 				可以理解为：rotate_matrix右乘的是目标坐标系的点，可以转换到原始参考系
	Eigen::Vector3f rotated_matrix1 = rotate_matrix * orin_point;
	std::cout << "Vector1: \n" << rotated_matrix1 << std::endl << std::endl;

	// 点的旋转方式2：将原始点（参考系下的）旋转 -theta 角之后的坐标，结果为 (5, 0, 0)
	//			    或者说固定点不动，将原始坐标系旋转 theta 角之后，点的新坐标表示。
	// 				可以理解为：rotate_matrix.inverse()右乘的是原始坐标系的点，可以转换到目标参考系
	Eigen::Vector3f rotated_matrix2 = rotate_matrix.inverse() * orin_point;
	std::cout << "Vector2: \n" << rotated_matrix2 << std::endl << std::endl;
	
	// std::cout << "Vector2: \n" <<  << std::endl;
	
	return 0;
}

