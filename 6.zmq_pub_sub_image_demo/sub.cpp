#include <iostream>
#include <zmq.hpp>
#include "image.pb.h"  // 生成的 Protocol Buffers 头文件
#include <opencv2/opencv.hpp>

int main() {
    // 打开 ZeroMQ 连接，接收 Image 消息
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PULL);
    socket.bind("tcp://*:5555");  // 使用与发送端相同的端口

    while (true) {
        zmq::message_t message;
        socket.recv(&message);
        std::cout << "recieve image!" << std::endl;

        // 解析接收到的消息为 Image 消息
        Image imageMsg;
        // imageMsg.ParseFromString(static_cast<const char*>(message.data()), message.size());
        imageMsg.ParseFromString(message.to_string());


        // 获取图像信息
        int width = imageMsg.width();
        int height = imageMsg.height();
        int channels = imageMsg.channels();
        std::string imageData = imageMsg.data();
        // std::string imageDataString(static_cast<char*>(imageMsg.data().c_str()), imageMsg.data().size()); 
        

        // 将图像数据还原为图像
        std::vector<uchar> buffer(imageData.begin(), imageData.end());
        cv::Mat receivedImage = cv::imdecode(buffer, cv::IMREAD_COLOR);
        // cv::Mat receivedImage = cv::imdecode(cv::Mat(imageData), cv::IMREAD_UNCHANGED);

        // 显示图像
        if (!receivedImage.empty()) {
            std::cout << "start showing the image!" << std::endl;
            std::cout << "Image width: " << width << ", Image height: " << height << ", Image channels: " << channels << std::endl; 
            cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);  
            cv::imshow("Received Image", receivedImage);
            cv::waitKey(0);  // 刷新 OpenCV 窗口
        } else {
            std::cerr << "Failed to decode received image data." << std::endl;
        }
    }

    return 0;
}
