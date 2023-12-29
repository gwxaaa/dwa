
#include "Model.h"
#include "ignition/math.hh"

Model::Model() : received(false) {
    // 创建 Gazebo 节点并初始化
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    // 订阅模型状态话题，并设置回调函数
    sub = node->Subscribe("~/model_states", &Model::modelStatesCallback, this);
}

void Model::modelStatesCallback(const gazebo::msgs::ModelPtr &msg) {
    for (int i = 0; i < msg->model_size(); ++i) {
        const gazebo::msgs::Model &currentModel = msg->model(i);

        // 获取当前模型的尺寸信息
        gazebo::msgs::Vector3d scale = currentModel.scale();
        double length = scale.x();
        double width = scale.y();
        double height = scale.z();

        // 获取当前模型的朝向信息
        gazebo::msgs::Quaternion orientation = currentModel.pose().orientation();
        ignition::math::Quaterniond quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        ignition::math::Vector3d euler = quat.Euler();
        double yaw = euler.Z();

        // 获取当前模型的位置信息
        gazebo::msgs::Vector3d position = currentModel.pose().position();
        double posX = position.x();
        double posY = position.y();
        double posZ = position.z();

        // 将接收到的信息转换为盒子
        convertedBox.size = { length, width, height };
        convertedBox.center = { posX, posY, posZ };
        convertedBox.yaw = yaw;

        // 设置接收状态为 true，表示已接收到消息
        received = true;
    }
}

Box Model::getConvertedBox() const {
    return convertedBox;
}

std::vector<Point> Model::calculatePerimeterAtZ(const Box &box, double zCoordinate) {
    double length = box.size.length;
    double width = box.size.width;
    double halfLength = length / 2.0;
    double halfWidth = width / 2.0;
    double cosYaw = cos(box.yaw);
    double sinYaw = sin(box.yaw);
    std::vector<Point> perimeterPoints;

    for (double angle = 0; angle < 2 * M_PI; angle += 0.1) {
        double x = halfLength * cos(angle);
        double y = halfWidth * sin(angle);
        double rotatedX = x * cosYaw - y * sinYaw + box.center.x;
        double rotatedY = x * sinYaw + y * cosYaw + box.center.y;
        perimeterPoints.push_back({rotatedX, rotatedY, zCoordinate});
    }
    return perimeterPoints;
}














// #include "Model.h"
// #include "ignition/math.hh"

// // 构造函数实现
// Model::Model() : received(false) {
//     // 创建 Gazebo 节点并初始化
//     node = gazebo::transport::NodePtr(new gazebo::transport::Node());
//     node->Init();
//     // 订阅模型状态话题，并设置回调函数
//     sub = node->Subscribe("~/model_states", &Model::modelStatesCallback, this);
// }

// // 获取模型尺寸信息的函数实现
// ModelSize Model::getModelSize(const std::string &modelName) {
//     // 等待回调函数接收到消息
//     while (!received) {
//         gazebo::common::Time::MSleep(100); // 通过短暂休眠等待回调函数接收消息
//     }
//     return receivedSize; // 返回接收到的模型尺寸信息
// }

// // 计算指定 Z 坐标下长方体一圈外围的点的函数实现
// std::vector<Point> Model::calculatePerimeterAtZ(const Box &box, double zCoordinate) {
//     double length = box.size.length;
//     double width = box.size.width;
//     double halfLength = length / 2.0;
//     double halfWidth = width / 2.0;
//     double cosYaw = cos(box.yaw);
//     double sinYaw = sin(box.yaw);
//     std::vector<Point> perimeterPoints;

//     for (double angle = 0; angle < 2 * M_PI; angle += 0.1) {
//         double x = halfLength * cos(angle);
//         double y = halfWidth * sin(angle);
//         double rotatedX = x * cosYaw - y * sinYaw + box.center.x;
//         double rotatedY = x * sinYaw + y * cosYaw + box.center.y;
//         perimeterPoints.push_back({rotatedX, rotatedY, zCoordinate});
//     }

//     return perimeterPoints;
// }

// // 订阅器回调函数实现，用于处理接收到的模型状态信息
// void Model::modelStatesCallback(const gazebo::msgs::ModelPtr &msg) {
//     for (int i = 0; i < msg->model_size(); ++i) {
//         const gazebo::msgs::Model &currentModel = msg->model(i);
//         std::string currentModelName = currentModel.name();
//         gazebo::msgs::Vector3d scale = currentModel.scale();
//         double length = scale.x();
//         double width = scale.y();
//         double height = scale.z();
//         // 获取当前模型的朝向信息
//         gazebo::msgs::Quaternion orientation = currentModel.pose().orientation();
//         ignition::math::Quaterniond quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
//         ignition::math::Vector3d euler = quat.Euler();
//         // 获取欧拉角的各个分量
//         double roll = euler.X();
//         double pitch = euler.Y();
//         double yaw = euler.Z();
//         // 获取当前模型的位置信息
//         gazebo::msgs::Vector3d position = currentModel.pose().position();
//         double posX = position.x();
//         double posY = position.y();
//         double posZ = position.z();
//         // 设置接收状态为 true，表示已接收到消息
//         received = true;
//     }
// }