

#ifndef MODEL_H
#define MODEL_H

#include <gazebo/msgs/model.pb.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
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
    // void modelStatesCallback(const gazebo::msgs::ModelPtr &msg);
    void modelStatesCallback(const boost::shared_ptr<const gazebo::msgs::Model> &msg);
    bool received; // 表示是否已经接收到消息
private:
    // 用于存储转换后的模型信息void Model::modelStatesCallback(const boost::shared_ptr<gazebo::msgs::ModelPtr>&msg) {
    Box convertedBox;
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
    std::mutex mutex;
    std::condition_variable cv;
};

#endif
// MODEL_H
