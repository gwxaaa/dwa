#include <iostream>
#include "ModelSizePlugin.h"

int main() {
    gazebo::ModelSizePlugin modelSizePlugin; // 创建插件实例

    // 假设 _modelPtr 是你的模型指针
    gazebo::physics::ModelPtr _modelPtr; // 假设这是你的模型指针

    // 加载模型信息
    modelSizePlugin.Load(_modelPtr, nullptr);

    // 获取模型信息
    double width = modelSizePlugin.GetModelWidth();
    double depth = modelSizePlugin.GetModelDepth();
    double height = modelSizePlugin.GetModelHeight();
    ignition::math::Vector3d position = modelSizePlugin.GetModelPosition();
    ignition::math::Quaterniond orientation = modelSizePlugin.GetModelOrientation();
    double yaw = modelSizePlugin.GetModelYaw();

    // 输出模型信息
    std::cout << "模型宽度：" << width << std::endl;
    std::cout << "模型深度：" << depth << std::endl;
    std::cout << "模型高度：" << height << std::endl;
    std::cout << "模型位置：" << position << std::endl;
    std::cout << "模型四元数：" << orientation << std::endl;
    std::cout << "模型Yaw信息:" << yaw << std::endl;

    return 0;
}