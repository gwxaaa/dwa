

#include "ModelSizePlugin.h"
#include "ignition/math.hh"
namespace gazebo
{

    void ModelSizePlugin::Load(const gazebo::physics::ModelStatePtr _modelstate, sdf::ElementPtr _sdf)
    {
        // 获取模型指针并检查
        model_ = modelstate;
        if (!model_)
        {
            gzerr << "Invalid model pointer\n";
            return;
        }

        // 获取模型名称
        std::string modelName = model_->GetName();

        // 获取模型的位置
        ignition::math::Pose3d modelPose = model_->WorldPose();
        ignition::math::Vector3d modelPosition = modelPose.Pos();
        gzmsg << "Model: " << modelName << ", Position: " << modelPosition << "\n";

        // 获取模型的朝向（四元数形式）
        ignition::math::Quaterniond modelOrientation = modelPose.Rot();
        gzmsg << "Model Orientation (Quaternion): " << modelOrientation << "\n";

        // 获取模型的尺寸信息
        physics::CollisionPtr collision = model_->GetLink()->GetCollision();
        if (collision)
        {
            ignition::math::Vector3d size = collision->GetSize();
            gzmsg << "Model Size (Length, Width, Height): " << size << "\n";
        }
        else
        {
            gzerr << "Failed to retrieve model's collision information\n";
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelSizePlugin)

} // namespace gazebo

// #include "ModelSizePlugin.h"

// namespace gazebo {

// void ModelSizePlugin::Load(physics::ModelState _model) {
//     // 获取模型指针
//     this->model = _model;

//     // 打印模型名称
//     std::string modelName = this->model->GetName();
//     std::cout << "Model name: " << modelName << std::endl;

//     // 获取模型的所有碰撞体
//     physics::Link_V links = this->model->GetLinks();
//     for (const auto &link : links) {
//         physics::Collision_V collisions = link->GetCollisions();
//         for (const auto &collision : collisions) {
//             // 获取碰撞体几何信息
//             physics::Collision::GeometryPtr geom = collision->GetShape();
//             if (geom != nullptr) {
//                 // 获取碰撞体的尺寸信息
//                 if (geom->HasType(physics::Collision::CYLINDER_SHAPE)) {
//                     auto cylinderGeom = std::dynamic_pointer_cast<physics::CylinderShape>(geom);
//                     if (cylinderGeom) {
//                         double radius = cylinderGeom->GetRadius();
//                         double length = cylinderGeom->GetLength();
//                         std::cout << "Cylinder - Radius: " << radius << ", Length: " << length << std::endl;
//                     }
//                 } else if (geom->HasType(physics::Collision::BOX_SHAPE)) {
//                     auto boxGeom = std::dynamic_pointer_cast<physics::BoxShape>(geom);
//                     if (boxGeom) {
//                         ignition::math::Vector3d size = boxGeom->GetSize();
//                         std::cout << "Box - SizeX: " << size.X() << ", SizeY: " << size.Y() << ", SizeZ: " << size.Z() << std::endl;
//                     }
//                 } else {
//                     // 处理其他类型的碰撞体（根据需要添加其他类型的几何形状）
//                 }
//             }
//         }
//     }
// }

// GZ_REGISTER_MODEL_PLUGIN(ModelSizePlugin)
// }  // namespace gazebo