
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>


std::string target_model_name; // 存储目标模型的名称

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // 获取所有模型的信息
    std::vector<std::string> model_names = msg->name;
    std::vector<geometry_msgs::Pose> model_poses = msg->pose;
    std::vector<geometry_msgs::Twist> model_twists = msg->twist;
    

    // 遍历所有模型
    for (size_t i = 0; i < model_names.size(); ++i)
    {
        std::string model_name = model_names[i];

        // 检查是否是目标模型
        if (model_name == target_model_name)
       {
 // 输出目标模型的信息
            ROS_INFO("Model Name: %s", model_name.c_str());
            ROS_INFO("Model Pose: x=%.2f, y=%.2f, z=%.2f", model_poses[i].position.x, model_poses[i].position.y, model_poses[i].position.z);
            ROS_INFO("Model Twist: linear x=%.2f, y=%.2f, z=%.2f", model_twists[i].linear.x, model_twists[i].linear.y, model_twists[i].linear.z);
            ROS_INFO("Model Twist: angular x=%.2f, y=%.2f, z=%.2f", model_twists[i].angular.x, model_twists[i].angular.y, model_twists[i].angular.z);
       } 
           
      
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot1_sub");
    ros::NodeHandle nh;

    // 获取目标模型的名称参数
    // target_model_name="model1.2";
     nh.param<std::string>("/target_model_name",target_model_name,"model1.2");
    // 订阅包含所有模型信息的主题
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    ros::Rate rate(1000);

    while (ros::ok()) {
        ros::spinOnce();  
        }
    return 0;
}