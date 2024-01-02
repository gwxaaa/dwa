
#include "ModelSizePlugin.h"
namespace gazebo {

    void ModelSizePlugin::Load(const gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->_model = _model;

        if (!_model) {
            gzerr << "无效的模型指针\n";
            return;
        }

        ignition::math::Pose3d modelPose = _model->WorldPose();
        modelPosition = modelPose.Pos();
        modelOrientation = modelPose.Rot();
        modelYaw = modelOrientation.Yaw();

        ignition::math::AxisAlignedBox bbox = _model->CollisionBoundingBox();
        modelWidth = bbox.Size().X();
        modelDepth = bbox.Size().Y();
        modelHeight = bbox.Size().Z();
        // std::cout << "宽度：" << modelWidth << " 深度：" << modelDepth << " 高度：" << modelHeight << std::endl;
        // std::cout << "位置：" << modelPosition << std::endl;
        // std::cout << "四元数：" << modelOrientation << std::endl;
        // std::cout << "Yaw 信息：" << modelYaw << std::endl;
    }
        double ModelSizePlugin::GetModelWidth() const
    {
        return modelWidth;
    }

    double ModelSizePlugin::GetModelDepth() const
    {
        return modelDepth;
    }

    double ModelSizePlugin::GetModelHeight() const
    {
        return modelHeight;
    }

    ignition::math::Vector3d ModelSizePlugin::GetModelPosition() const
    {
        return modelPosition;
    }

    ignition::math::Quaterniond ModelSizePlugin::GetModelOrientation() const
    {
        return modelOrientation;
    }

    double ModelSizePlugin::GetModelYaw() const
    {
        return modelYaw;
    }
}
// //#include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include "gazebo/physics/BoxShape.hh"
// #include "ModelSizePlugin.h"
// #include "ignition/math.hh"
// namespace gazebo
// {

//     void ModelSizePlugin::Load(const gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
//     {
//         // 获取模型指针并检查
//         this->_model = _model;
//         if (!_model)
//         {
//             gzerr << "Invalid model pointer\n";
//             return;
//         }

//         // 获取模型名称
//         std::string modelName = _model->GetName();

//         // 获取模型的位置
//         ignition::math::Pose3d modelPose = _model->WorldPose();
//         ignition::math::Vector3d modelPosition = modelPose.Pos();
//         gzmsg << "Model: " << modelName << ", Position: " << modelPosition << "\n";

//         // 获取模型的朝向（四元数形式）
//         ignition::math::Quaterniond modelOrientation = modelPose.Rot();
//         gzmsg << "Model Orientation (Quaternion): " << modelOrientation << "\n";

//         // 获取模型尺寸信息
//         ignition::math::AxisAlignedBox bbox = _model->CollisionBoundingBox();
//         double modelWidth = bbox.Size().X();
//         double modelDepth = bbox.Size().Y();
//         double modelHeight = bbox.Size().Z();
//         // 获取模型的尺寸信息
//         std::cout << "Position: x=" << modelPosition.X() << " y=" << modelPosition.Y() << " z=" << modelPosition.Z() << std::endl;
//         std::cout << "Orientation: x=" << modelOrientation.X() << " y=" << modelOrientation.Y() << " z=" << modelOrientation.Z() << " w=" << modelOrientation.W() << std::endl;
//         // 输出模型尺寸信息
//         std::cout << "Width: " << modelWidth << " Depth: " << modelDepth << " Height: " << modelHeight << std::endl;
//     } // namespace gazebo
// }
// GZ_REGISTER_MODEL_PLUGIN(gazebo::ModelSizePlugin)