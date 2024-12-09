#include <opencv2/opencv.hpp>  
#include <iostream>  
  
int main(int argc, char** argv) {  
    // 确保有一个命令行参数指定图像文件  
    if (argc != 2) {  
        std::cout << "Usage: display_image <Image_Path>" << std::endl;  
        return -1;  
    }  
  
    // 读取图像文件  
    cv::Mat image;  
    image = cv::imread(argv[1], cv::IMREAD_COLOR);  
  
    // 检查图像是否正确加载  
    if (!image.data) {  
        std::cout << "No image data" << std::endl;  
        return -1;  
    }  
  
    // 创建一个窗口来显示图像  
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);  
  
    // 在窗口中显示图像  
    cv::imshow("Display Image", image);  
  
    // 等待按键事件  
    cv::waitKey(0);  
  
    return 0;  
}
