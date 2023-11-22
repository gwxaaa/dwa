

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Twist.h>

std::string target_model_name;
ros::Publisher model_state_pub;

double new_x, new_y, new_z;
double speed ; // 移动速度
double time_interval ; // 时间间隔

geometry_msgs::Pose current_position;
int num_steps=10;
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
            // 保存目标模型的当前位置
            current_position = model_poses[i];
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
    ros::init(argc, argv, "sub_pub");
    ros::NodeHandle nh("~");

    nh.param<std::string>("target_model_name", target_model_name, "model2.1");
    nh.param<double>("new_x", new_x, 0.0);
    nh.param<double>("new_y", new_y, 0.0);
    double speed; 
    nh.param("speed", speed, 0.1); // 从ROS参数服务器中读取时间间隔参数，如果不存在，使用默认值0.1 
    double time_interval; 
    nh.param("time_interval", time_interval, 0.1);
 


    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    ros::Rate rate(1.0);

    while (ros::ok()) {
        // 计算每步的位移
        double dx = (new_x - current_position.position.x) / num_steps;
        double dy = (new_y - current_position.position.y) / num_steps;

        for (int step = 0; step < num_steps; ++step) {
            current_position.position.x += dx;
            current_position.position.y += dy;

            gazebo_msgs::ModelState model_state;
            model_state.model_name = target_model_name;
            model_state.pose = current_position;
            
            model_state_pub.publish(model_state);

            // 休眠一段时间间隔
            ros::Duration(time_interval).sleep();

            // 如果到达目标位置，退出循环
            if (fabs(current_position.position.x - new_x) < 0.01 && fabs(current_position.position.y - new_y) < 0.01) {
                break;
            }
        }

        ros::spinOnce();
    }

    return 0;
}

