#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "model_control_publisher");
    ros::NodeHandle nh;

    // 创建发布者，发布到/gazebo/set_model_state话题
    ros::Publisher model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    // 设置发布频率
    ros::Rate rate(1.0);  // 1 Hz

    int num_models;
    std::cout << "Enter the number of models to control: ";
    std::cin >> num_models;

    for (int i = 0; i < num_models; ++i) {
        // 创建一个消息对象
        gazebo_msgs::ModelState model_state;

        // 读取用户输入的模型名称
        std::string model_name;
        std::cout << "Enter the model name for model " << i + 1 << ": ";
        std::cin >> model_name;
        model_state.model_name = model_name;

        // 读取用户输入的新位置信息
        double new_x, new_y, new_z;
        std::cout << "Enter the new X position for model " << model_name << ": ";
        std::cin >> new_x;
        std::cout << "Enter the new Y position for model " << model_name << ": ";
        std::cin >> new_y;
        std::cout << "Enter the new Z position for model " << model_name << ": ";
        std::cin >> new_z;

        model_state.pose.position.x = new_x;
        model_state.pose.position.y = new_y;
        model_state.pose.position.z = new_z;

        // 读取用户输入的新线性速度信息
        double new_linear_x, new_linear_y, new_linear_z;
        std::cout << "Enter the new X linear velocity for model " << model_name << ": ";
        std::cin >> new_linear_x;
        std::cout << "Enter the new Y linear velocity for model " << model_name << ": ";
        std::cin >> new_linear_y;
        std::cout << "Enter the new Z linear velocity for model " << model_name << ": ";
        std::cin >> new_linear_z;

        model_state.twist.linear.x = new_linear_x;
        model_state.twist.linear.y = new_linear_y;
        model_state.twist.linear.z = new_linear_z;

        // 读取用户输入的新角速度信息
        double new_angular_x, new_angular_y, new_angular_z;
        std::cout << "Enter the new X angular velocity for model " << model_name << ": ";
        std::cin >> new_angular_x;
        std::cout << "Enter the new Y angular velocity for model " << model_name << ": ";
        std::cin >> new_angular_y;
        std::cout << "Enter the new Z angular velocity for model " << model_name << ": ";
        std::cin >> new_angular_z;

        model_state.twist.angular.x = new_angular_x;
        model_state.twist.angular.y = new_angular_y;
        model_state.twist.angular.z = new_angular_z;

        // 发布模型状态消息
        model_state_pub.publish(model_state);
    }

    ros::spin();

    return 0;
}

