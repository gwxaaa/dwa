


// #ifndef MODELSIZEGET_H
// #define MODELSIZEGET_H

// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>
// #include <iostream>
// #include <string>
// #include <mutex>
// #include <condition_variable>
// #include <gazebo_msgs/ModelStates.h>
// // 定义一个表示模型的尺寸信息的结构体
// struct ModelSize
// {
//     double length, width, height;
// };

// class ModelSizeGet
// {
// public:
//     ModelSizeGet();
//     ModelSize getModelSize(const std::string &modelName);

// private:
//     gazebo::transport::NodePtr node{new gazebo::transport::Node()};
//     std::mutex mutex;
//     std::condition_variable cv;
//     bool received = false;
// };

// #endif // MODELSIZEGETTER_H
#ifndef MODELSIZEPLUGIN_HH
#define MODELSIZEPLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/BoxShape.hh"

namespace gazebo
{

    class ModelSizePlugin : public WorldPlugin
    {
    public:
        void gazebo::physics::load( WorldPtr  _world ,sdf::ElementPtr _sdf) ;

    // private:
    //     gazebo::physics::WorldPtr  _world;

    };

} // namespace gazebo

#endif // MODELSIZEPLUGIN_HH
