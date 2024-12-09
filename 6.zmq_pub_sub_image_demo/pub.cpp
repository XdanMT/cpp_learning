#include <zmq.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "image.pb.h"  // 生成的 Protocol Buffers 头文件

int main() {
    // 读取图像文件
    cv::Mat image = cv::imread("cat.png");
    if (image.empty()) {
        std::cerr << "Failed to read image file." << std::endl;
        return 1;
    }

    // 创建 Image 消息
    Image imageMsg;
    imageMsg.set_width(image.cols);
    imageMsg.set_height(image.rows);
    imageMsg.set_channels(image.channels());

    // 将图像数据转换为字符串并设置到 Image 消息中
    std::vector<uchar> buffer;
    cv::imencode(".png", image, buffer);
    std::string imageData(buffer.begin(), buffer.end());
    imageMsg.set_data(imageData);

    // 打开 ZeroMQ 连接，发送 Image 消息
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);
    socket.connect("tcp://192.168.8.217:5555");

    // 将 Image 消息序列化并发送
    std::string serializedData;
    imageMsg.SerializeToString(&serializedData);

    zmq::message_t message(serializedData.size());
    memcpy(message.data(), serializedData.c_str(), serializedData.size());
    socket.send(message);

    std::cout << "Image information sent successfully." << std::endl;

    return 0;
}

