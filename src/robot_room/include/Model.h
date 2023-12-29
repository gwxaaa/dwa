

#ifndef MODEL_H
#define MODEL_H

#include <gazebo/msgs/model.pb.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <string>
#include <vector>
struct Vector3
{
    double x, y, z;
};

struct Size
{
    double length, width, height;
};

struct Box
{
    Size size;
    Vector3 center;
    double yaw;
};

struct Point
{
    double x, y, z;
};

class Model
{
public:
    Model();
    // 函数用于外部获取转换后的模型信息
    Box getConvertedBox() const;
    // 计算指定 Z 坐标下长方体一圈外围的点
    std::vector<Point> calculatePerimeterAtZ(const Box &box, double zCoordinate);
    void modelStatesCallback(const gazebo::msgs::ModelPtr &msg);

private:
    // 用于存储转换后的模型信息
    Box convertedBox;
    bool received; // 表示是否已经接收到消息
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
};

#endif // MODEL_H

// #ifndef MODEL_H
// #define MODEL_H

// #include <gazebo/msgs/model.pb.h>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>
// #include <ignition/math.hh>
// #include <string>
// #include <vector>

// // 定义一个表示点坐标的结构体
// struct Point {
//     double x, y, z;
// };

// // 定义一个表示模型的尺寸信息的结构体
// struct ModelSize {
//     double length, width, height;
// };

// // 定义 Box 结构体
// struct Box {
//     ModelSize size;
//     Point center;
//     double yaw;
// };

// class Model {
// public:
//     Model(); // 构造函数声明
//     ModelSize getModelSize(const std::string &modelName); // 获取模型尺寸信息的函数声明
//     std::vector<Point> calculatePerimeterAtZ(const Box &box, double zCoordinate); // 计算外围点的函数声明

// private:
//     gazebo::transport::NodePtr node; // Gazebo 节点指针
//     gazebo::transport::SubscriberPtr sub; // 订阅器指针
//     bool received; // 消息接收标志
//     ModelSize receivedSize; // 接收到的模型尺寸信息
//     void modelStatesCallback(const gazebo::msgs::ModelPtr &msg);
// };

// #endif // MODEL_H