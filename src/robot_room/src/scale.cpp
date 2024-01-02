#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class ModelSizePlugin : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            // 输出模型名称
            std::string modelName = _model->GetName();
            std::cout << "Model Name: " << modelName << std::endl;

            // 获取模型所有链接
            physics::Link_V links = _model->GetLinks();

            // 循环遍历链接
            for(auto& link : links)
            {
                // 获取链接的碰撞体列表
                physics::Collision_V collisions = link->GetCollisions();

                // 遍历碰撞体列表
                for(auto& collision : collisions)
                {
                    // 获取碰撞体的边界盒
                    ignition::math::Box bbox = collision->BoundingBox();
                    double width = bbox.Max().X() - bbox.Min().X();
                    double depth = bbox.Max().Y() - bbox.Min().Y();
                    double height = bbox.Max().Z() - bbox.Min().Z();

                    // 输出碰撞体尺寸
                    std::cout << "Collision Name: " << collision->GetName() << std::endl;
                    std::cout << "Width: " << width << " Depth: " << depth << " Height: " << height << std::endl;
                }
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelSizePlugin)
}








// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>

// class ModelSizePlugin : public gazebo::ModelPlugin {
// public:
//     void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
//         // 获取模型指针
//         this->model = _model;

//         // 获取模型名称
//         std::string modelName = this->model->GetName();

//         // 获取模型的 CollisionPtr（假设模型有一个名为 "collision" 的碰撞体）
//         gazebo::physics::CollisionPtr collision = this->model->GetLink()->GetCollision("collision");

//         if (collision) {
//             // 获取碰撞盒的尺寸
//             gazebo::math::Box bbox = collision->GetBoundingBox();
//             double width = bbox.GetXLength();
//             double depth = bbox.GetYLength();
//             double height = bbox.GetZLength();

//             // 输出模型尺寸
//             std::cout << "Model Name: " << modelName << std::endl;
//             std::cout << "Width: " << width << " Depth: " << depth << " Height: " << height << std::endl;
//         } else {
//             std::cerr << "No collision named 'collision' found for model: " << modelName << std::endl;
//         }
//     }

// private:
//     gazebo::physics::ModelPtr model;
// };

// GZ_REGISTER_MODEL_PLUGIN(ModelSizePlugin)