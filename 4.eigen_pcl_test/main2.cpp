// 手写一个获取旋转矩阵的操作(和Eigen的Eigen::AngleAxisf接口输出对应)，并做了对比输出验证

#include <iostream>
#include <string>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

void matrixVectorMultiply(const float* A, const float* B, float* result) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            result[i] += A[i * 3 + j] * B[j];
        }
    }
}


void matrix_multiply(float* A, float* B) {
    float result[9];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += A[i * 3 + k] * B[k * 3 + j];
            }
            result[i * 3 + j] = sum;
        }
    }

    for (int i = 0; i < 9; ++i) {
        A[i] = result[i];
    }
}


void get_rotation_matrix(const float* angle, float* rotation_matrix) {
    /*
        输入的a, b, c是参考坐标系转动到目标坐标系弧度，不是角度；
        得到的旋转矩阵是参考坐标系的点转换到目标坐标系的直乘矩阵(效果等价于把参考坐标系的点旋转angle度): p

    */

    // Step 1: Initialize identity matrix
    float a = angle[0];
    float b = angle[1];
    float c = angle[2];

    // Step 2: Compute rotation matrices
    float Rx[9] = {1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a)};
    float Ry[9] = {cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b)};
    float Rz[9] = {cos(c), -sin(c), 0, sin(c), cos(c), 0, 0, 0, 1};
    // float Rx[9] = {1, 0, 0, 0, cos(a), sin(a), 0, -sin(a), cos(a)};
    // float Ry[9] = {cos(b), 0, -sin(b), 0, 1, 0, sin(b), 0, cos(b)};
    // float Rz[9] = {cos(c), sin(c), 0, -sin(c), cos(c), 0, 0, 0, 1};

    // Step 3: Combine rotation matrices
    for (int i = 0; i < 9; ++i) {
        rotation_matrix[i] = Rz[i];
    }

    matrix_multiply(rotation_matrix, Ry);
    matrix_multiply(rotation_matrix, Rx);
}

// void transposeMatrix(float* matrix, int nxn_size) {
//     /*
//         nxn_size指的是，这个数组是几乘几的，例如 3*3 的数组，nxn_size等于3
//     */
//     for (int i = 0; i < nxn_size; ++i) {
//         for (int j = i+1; j < nxn_size; ++j) {
//             std::swap(matrix[i*nxn_size + j], matrix[j*nxn_size + i]);
//         }
//     }
// }

void transposeMatrix(float* matrix, int nxn_size) {
    float temp[nxn_size * nxn_size];

    // 计算转置矩阵并存储在temp中
    for (int i = 0; i < nxn_size; ++i) {
        for (int j = 0; j < nxn_size; ++j) {
            temp[i*nxn_size + j] = matrix[j*nxn_size + i];
        }
    }

    // 将temp中的值复制回matrix
    for (int i = 0; i < 9; ++i) {
        matrix[i] = temp[i];
    }
}

void printMatrix(float* matrix, int n, int m) {
	std::cout << "The initialized affine matrix: \n";
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j < m; ++j) {
            std::cout << matrix[i * m + j] << " ";
        }
        std::cout << std::endl;
    }
	std::cout << std::endl;
}


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
    std::cout << "Eigen output rotation matrix: \n" << rotate_matrix << std::endl << std::endl;


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


	// 手动方式实现旋转矩阵的获取
	float input_point[3] = {3.0, 4.0, 0.0};
	float input_point2[3] = {5, 0.0, 0.0};

	float z_start_angle[3] = {0.0, 0.0, start_orientation};
	float* start_rotation_matrix = new float[9];
	get_rotation_matrix(z_start_angle, start_rotation_matrix);		// 和Eigen的输出的接口一样
    printMatrix(start_rotation_matrix, 3, 3);
	
	// 点的旋转方式1：将原始点（参考系下的）旋转 theta 角之后的坐标，结果为 (-1.4, 4.8, 0)
	float* rotated_point1 = new float[3];
	matrixVectorMultiply(start_rotation_matrix, input_point, rotated_point1);
	std::cout << "Vector2: \n" << rotated_point1[0] << std::endl
							   << rotated_point1[1] << std::endl 
							   << rotated_point1[2] << std::endl;


	// 点的旋转方式2：将原始点（参考系下的）旋转 -theta 角之后的坐标，结果为 (5, 0, 0)
	float* rotated_point2 = new float[3];
	transposeMatrix(start_rotation_matrix, 3);		// 转置结果等于求逆
	matrixVectorMultiply(start_rotation_matrix, input_point, rotated_point2);
	std::cout << "Vector2: \n" << rotated_point2[0] << std::endl
							   << rotated_point2[1] << std::endl 
							   << rotated_point2[2] << std::endl;

	
	return 0;
}