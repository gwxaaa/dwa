#include "ModelSizeGet.h"
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

ModelSizeGet::ModelSizeGet(const std::string& modelName) : modelName_(modelName) {}

void ModelSizeGet::GetModelSize() {
    // 初始化 Gazebo 客户端
    gazebo::client::setup();

    // 连接到 Gazebo 服务器
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // 获取模拟器中的世界
    gazebo::physics::World world = gazebo::physics::get_world("default");
    // 获取模型对象
    gazebo::physics::Model model = world->ModelByName(modelName_);

    if (!model) {
        std::cerr << "未找到模型！" << std::endl;
        return;
    }

    // 获取模型的名称
    std::string modelName = model->GetName();
    std::cout << "模型名称: " << modelName << std::endl;

    // 获取模型的边界框
    gazebo::math::Box boundingBox = model->BoundingBox();

    // 获取模型的尺寸
    gazebo::math::Vector3 modelSize = boundingBox.GetSize();
    std::cout << "模型尺寸 (长宽高): " << modelSize << std::endl;

    // 清理并关闭 Gazebo 客户端
    gazebo::client::shutdown();
}