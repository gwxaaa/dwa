
#include "Model.h"
#include "ignition/math.hh"

Model::Model() : received(false)
{
    // 创建 Gazebo 节点并初始化

    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    // 订阅模型状态话题，并设置回调函数
    const std::string topic = "~/default/model5.1/link_1/wrench";
    gazebo::transport::SubscriberPtr sub = node->Subscribe(topic, &Model::modelStatesCallback, this);
}
void Model::modelStatesCallback(const boost::shared_ptr<const gazebo::msgs::Model> &msg)
{
    // void Model::modelStatesCallback(const gazebo::msgs::ModelPtr& msg) {
      //  
    for (int i = 0; i < msg->model_size(); ++i)
    {
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
        convertedBox.size = {length, width, height};
        convertedBox.center = {posX, posY, posZ};
        convertedBox.yaw = yaw;

        // 设置接收状态为 true，表示已接收到消息
        received = true;
    }
}

Box Model::getConvertedBox() const
{
    return convertedBox;
}

std::vector<Point> Model::calculatePerimeterAtZ(const Box &box, double zCoordinate)
{
    double length = box.size.length;
    double width = box.size.width;
    double halfLength = length / 2.0;
    double halfWidth = width / 2.0;
    double cosYaw = cos(box.yaw);
    double sinYaw = sin(box.yaw);
    std::vector<Point> perimeterPoints;

    for (double angle = 0; angle < 2 * M_PI; angle += 0.1)
    {
        double x = halfLength * cos(angle);
        double y = halfWidth * sin(angle);
        double rotatedX = x * cosYaw - y * sinYaw + box.center.x;
        double rotatedY = x * sinYaw + y * cosYaw + box.center.y;
        // v2 = v1 * R + b;
        perimeterPoints.push_back({rotatedX, rotatedY, zCoordinate});
    }
    return perimeterPoints;
}

