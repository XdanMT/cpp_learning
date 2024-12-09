#include <iostream>
#include <string>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>


int main(){

    Eigen::Vector3f angular_rate_(1.0, 2.0, 3.0);
    float* angular = angular_rate_.data();

    std::cout << "angular_rate_: \n" << angular_rate_ << std::endl;
    std::cout << std::endl;

    std::cout << "angular: \n";
    for (int i = 0; i < 3; i++){
        std::cout << angular[i] << "\n";
    }
    std::cout << std::endl;

    angular[0] = 150.0;
    angular[1] = 10.0;
    angular[2] = 130.0;

    std::cout << "angular_rate_: \n" << angular_rate_ << std::endl;
    std::cout << std::endl;
    


}