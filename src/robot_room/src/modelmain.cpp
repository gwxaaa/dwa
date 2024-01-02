#include <iostream>
#include "Model.h"
#include <ros/ros.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "modelmain");
    ros::NodeHandle nh;


// int main() {
    // 创建一个模型对象
    Model model;

    // 循环等待接收消息，直到接收到消息为止
    while (!model.received);

    // 获取转换后的模型信息
    Box convertedBox = model.getConvertedBox();

    // 计算指定 Z 坐标下长方体一圈外围的点集
    double zCoordinate = 0.0; // 指定 Z 坐标
    std::vector<Point> perimeterPoints = model.calculatePerimeterAtZ(convertedBox, zCoordinate);

    // 输出点集
    
    std::cout << "Perimeter Points at Z = " << zCoordinate << ":" << std::endl;
    for (const auto &point : perimeterPoints) {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}