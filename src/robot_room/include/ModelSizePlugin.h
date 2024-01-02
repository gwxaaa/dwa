

#ifndef MODELSIZEPLUGIN_H
#define MODELSIZEPLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/BoxShape.hh"
#include "ignition/math.hh"

namespace gazebo {

    class ModelSizePlugin : public ModelPlugin {
    public:
        void Load(const gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) ;
        // 获取模型宽度
        double GetModelWidth() const ;
        // 获取模型深度
        double GetModelDepth() const ;
        // 获取模型高度
        double GetModelHeight() const ;
        // 获取模型位置信息
        ignition::math::Vector3d GetModelPosition() const ;
        // 获取模型四元数
        ignition::math::Quaterniond GetModelOrientation() const ;
        // 获取模型Yaw信息
        double GetModelYaw() const ;

    private:
        gazebo::physics::ModelPtr _model;
        double modelWidth;
        double modelDepth;
        double modelHeight;
        ignition::math::Vector3d modelPosition;
        ignition::math::Quaterniond modelOrientation;
        double modelYaw;
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelSizePlugin)
}

#endif // MODELSIZEPLUGIN_H




// #ifndef MODELSIZEPLUGIN_HH
// #define MODELSIZEPLUGIN_HH

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include "gazebo/physics/BoxShape.hh"

// namespace gazebo
// {

//     class ModelSizePlugin : public ModelPlugin
//     {
//     public:
//         void Load(physics::ModelPtr  _model,sdf::ElementPtr _sdf) ;

//     private:
//         gazebo::physics::ModelPtr  _model;


//     };

// } // namespace gazebo

// #endif // MODELSIZEPLUGIN_HH
