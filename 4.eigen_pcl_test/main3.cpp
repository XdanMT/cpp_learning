// 手动实现点云的齐次变换操作，并和Eigen方式的齐次变换结果进行了对比验证

#include <iostream>
#include <string>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>

void matrixVectorMultiply(const float* A, const float* B, float* result) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            result[i] += A[i * 3 + j] * B[j];
        }
    }
}

void matrixVectorMultiply_m_n_k(const float* A, const float* B, int m, int n, int k, float* C) {
    for(int i = 0; i < m; ++i) {
        for(int j = 0; j < k; ++j) {
            float tmp = 0.0;
            for(int l = 0; l < n; ++l) {
                tmp += A[i*n + l] * B[l*k + j];
            }
			C[i*k + j] = tmp;
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


void get_affine_matrix(float* start_rotation_matrix, float* affine_matrix){

	// 初始化齐次变换矩阵为单位矩阵
	for (int i = 0; i < 16; ++i) {
		affine_matrix[i] = 0.0;
	}
	affine_matrix[0] = 1.0;
	affine_matrix[5] = 1.0;
	affine_matrix[10] = 1.0;
	affine_matrix[15] = 1.0;

	// 将旋转矩阵赋值给齐次变换矩阵的旋转部分
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			affine_matrix[i * 4 + j] = start_rotation_matrix[i * 3 + j];
		}
	}

}


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
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j < m; ++j) {
            std::cout << matrix[i * m + j] << " ";
        }
        std::cout << std::endl;
    }
	std::cout << std::endl;
}


int main(){

    /* ***************************************************************************************************************************** */
	// Eigne方式实现点云的齐次变换

    // 创建点云对象
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->resize(2);
    cloud->height = 1;      // 单层点云
    cloud->width = 2;       // 表示一层有两个点
    
    // 初始化第一个点
    cloud->points[0].x = 3.0;
    cloud->points[0].y = 4.0;
    cloud->points[0].z = 0.0;
    cloud->points[0].r = 0;
    cloud->points[0].g = 0;
    cloud->points[0].b = 255;

    // 初始化第二个点
    cloud->points[1].x = 0.0;
    cloud->points[1].y = 5.0;
    cloud->points[1].z = 0.0;
	cloud->points[1].r = 255;
    cloud->points[1].g = 0;
    cloud->points[1].b = 0;

	// 初始化点的
	Eigen::Vector3f orin_point(3.0, 4.0, 0.0);
	Eigen::Vector3f orin_point2(5.0, 0.0, 0.0);


	// 找到该点的方向 theta 度，（将原始参考坐标系，绕z轴旋转到目标坐标系后使得xoy平面重合，经过的角度）
	float start_orientation = atan2(cloud->points[0].y, cloud->points[0].x); 
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ()); 
	Eigen::Matrix3f rotate_matrix = t_V.matrix();
    std::cout << "Eigen affine_matrix: \n" << rotate_matrix << std::endl << std::endl;

    transform.rotate((rotate_matrix.inverse()));            // 求逆，因为要让点转 -start_orientation 角度
	transform.translate(Eigen::Vector3f (0.0, 0.0, 0.0));
	std::cout << "Eigen assigned affine matrix: \n" << transform.matrix() << std::endl << std::endl;

	// 将所有的点旋转 -start_orientation 角度
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*cloud, *cloud, transform);   		// 覆盖输出

    // 打印Eigen方式转化后的点的矩阵
    std::cout << "Eigen rotated_two_points:" << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i){
        std::cout << "Point " << i + 1 << ": (" << cloud->points[i].x << ", "
                  << cloud->points[i].y << ", " << cloud->points[i].z << ")" << std::endl;
    }
    std::cout << std::endl << std::endl;


	
	// 可视化Eigen最后变换后的点云
    if (false){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

        // 添加点云、坐标轴
        viewer->addPointCloud(cloud, "cloud");
        viewer->addCoordinateSystem(1.0, "coordinate", 0);

        // 设置点的大小
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "cloud");

        // 显示点云
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
    }
	
    /* ***************************************************************************************************************************** */
	// 手动实现点云的齐次变换
	float input_two_points[8] = {3.0, 0.0, 
								 4.0, 5.0, 
								 0.0, 0.0,
								 0.0, 0.0};

	// 获得旋转矩阵
	float z_start_angle[3] = {0.0, 0.0, start_orientation};
	float* start_rotation_matrix = new float[9];
	get_rotation_matrix(z_start_angle, start_rotation_matrix);		// 和Eigen的输出的接口一样
    std::cout << "Manual start_rotation_matrix: \n";
	printMatrix(start_rotation_matrix, 3, 3);
    transposeMatrix(start_rotation_matrix, 3);		                // 求逆，因为要让点转 -start_orientation 角度
    

	// 获得齐次矩阵
	float* affine_matrix = new float[16];
	get_affine_matrix(start_rotation_matrix, affine_matrix);        // 默认平移矩阵为 [0, 0, 0]
    std::cout << "Manual affine_matrix: \n";
	printMatrix(affine_matrix, 4, 4);

    // 点云进行其次变换
	float* rotated_two_points = new float[8];
	matrixVectorMultiply_m_n_k(affine_matrix, input_two_points, 4, 4, 2, rotated_two_points);

    
    // 输出计算得到的两个点的坐标
    std::cout << "Manual rotated_two_points: \n";
    printMatrix(rotated_two_points, 4, 2);
    for (int i = 0; i < 2; i++){
        std::cout << "Point " << i 
                  << " X: " << rotated_two_points[0 * 2 + i]
                  << ", Y: " << rotated_two_points[1 * 2 + i]
                  << ", Z: " << rotated_two_points[2 * 2 + i]
                  << std::endl;
    }


	delete[] start_rotation_matrix;
	delete[] affine_matrix;
	delete[] rotated_two_points;

	return 0;
}