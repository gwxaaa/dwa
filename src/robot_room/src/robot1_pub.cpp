#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot1_pub");
    ros::NodeHandle nh;

    // 创建发布者，发布到/gazebo/set_model_state话题
    ros::Publisher model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    // 设置发布频率
    ros::Rate rate(1.0);  // 1 Hz

    std::string model_name;
    double new_x, new_y, new_z, new_linear_x, new_linear_y, new_linear_z, new_angular_x, new_angular_y, new_angular_z;

    // 获取参数值
    ros::param::param<std::string>("model_name", model_name, "your_model_name");
    ros::param::param<double>("new_x", new_x, 1.0);
    ros::param::param<double>("new_y", new_y, 2.0);
    ros::param::param<double>("new_z", new_z, 0.0);
    ros::param::param<double>("new_linear_x", new_linear_x, 0.1);
    ros::param::param<double>("new_linear_y", new_linear_y, 0.0);
    ros::param::param<double>("new_linear_z", new_linear_z, 0.0);
    ros::param::param<double>("new_angular_x", new_angular_x, 0.0);
    ros::param::param<double>("new_angular_y", new_angular_y, 0.0);
    ros::param::param<double>("new_angular_z", new_angular_z, 0.0);

    while (ros::ok()) {
        gazebo_msgs::ModelState model_state;
        model_state.model_name = model_name;
        model_state.pose.position.x = new_x;
        model_state.pose.position.y = new_y;
        model_state.pose.position.z = new_z;
        model_state.twist.linear.x = new_linear_x;
        model_state.twist.linear.y = new_linear_y;
        model_state.twist.linear.z = new_linear_z;
        model_state.twist.angular.x = new_angular_x;
        model_state.twist.angular.y = new_angular_y;
        model_state.twist.angular.z = new_angular_z;

        model_state_pub.publish(model_state);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}