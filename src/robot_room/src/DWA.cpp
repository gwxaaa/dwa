#include "DWA.h"
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <KinematicModel.h>
DWAPlanner::DWAPlanner(const geometry_msgs::Pose &target_pose, std::vector<Obstacle> obstacles,
                       double max_linear_speed, double max_angular_speed, double time, double num,
                       const geometry_msgs::Pose &current_pose)
    : target_pose(target_pose), obstacles(obstacles), max_distance(1.0), some_threshold(some_threshold), max_linear_speed(max_linear_speed), max_angular_speed(max_angular_speed), time(time), num(num)
{
}
DWAPlanner::~DWAPlanner()
{
}
const geometry_msgs::Twist &DWAPlanner::FindBestTwist(const geometry_msgs::Pose &current_pose)
{
  // 初始化最佳速度和最佳分数
  best_twist = current_twist;
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::Twist> twist_combinations = GenerateTwists();
  double max_distance = FindMaxDistance(twist_combinations, current_pose);
  // std::cout << "Max Distance: " << max_distance << std::endl;
  // 创建一个临时变量来存储最佳速度
  geometry_msgs::Twist best_temp_twist = best_twist;
  geometry_msgs::Pose best_final_pose = final_pose;
  double best_final_score = -0.1;
  // 循环评估速度组合
  for (const auto &current_twist : twist_combinations)
  {
    // 预测最终姿态
    final_pose = PredictPose(current_pose, current_twist, time);
    // 检查是否接近目标点
    double distance_to_target = CalculateDistance(final_pose);
    // 检查碰撞
    double collision = CalculateCollision(final_pose);
    // 如果当前组合的分数更高，更新最佳分数和速度角速度
    double score = CalculateScore(current_twist);
    // std::cout << "Score: " << score << std::endl;
    // std::cout << "Score1: " << best_final_score << std::endl;
    // std::cout << "twist: " << current_twist<< std::endl;
    //    std::cout << " - Best Final Pose: x = " << final_pose.position.x << ", y = " << final_pose.position.y <<
    // std::endl;
    if (score > best_final_score)
    {
      best_final_score = score;
      best_temp_twist = current_twist;
      best_final_pose = final_pose;
    }
  }
  // 更新最佳速度
  best_twist = best_temp_twist;
  final_pose = best_final_pose;
  return best_twist;
}

std::vector<geometry_msgs::Twist> DWAPlanner::GenerateTwists()
{
  std::vector<geometry_msgs::Twist> twist_combinations; // 存储多组速度角速度组合
  double linear_speed_increment = (2 * max_linear_speed) / static_cast<double>(num - 1);
  double angular_speed_increment = (2 * max_angular_speed) / static_cast<double>(num - 1);
  for (int i = 0; i < num; i++)
  {
    for (int j = 0; j < num; j++)
    {
      // 创建速度组合
      geometry_msgs::Twist current_twist;
      current_twist.linear.x = -max_linear_speed + i * linear_speed_increment;
      current_twist.angular.z = -max_angular_speed + j * angular_speed_increment;
      twist_combinations.push_back(current_twist); // 存储当前组合
    }
  }

  return twist_combinations;
}

geometry_msgs::Pose DWAPlanner::PredictPose(const geometry_msgs::Pose &current_pose, const geometry_msgs::Twist &twist,
                                            double time)
{
  KinematicModel kinematic_model(current_pose, twist);
  geometry_msgs::Pose final_pose = kinematic_model.calculateNewPosition(time);
  return final_pose;
}
// 对于点和其他信息，需要合并处理，找到距离最近的障碍物，进行避障
double DWAPlanner::CalculateCollision(const geometry_msgs::Pose &final_pose)
{
  // 初始化参数
  double min_collision_distance_model = 1.1;         // model的最小碰撞距离
  double avoidance_distance_model = 1.5;           // model的避障范围起始距离
  //double collision_distance_threshold_model = 1.6; // model的碰撞阈值

  double min_collision_distance_other = 0.6;       // 其他的最小碰撞距离
  double avoidance_distance_other = 0.8;           // 其他的避障范围起始距离
 // double collision_distance_threshold_other = 0.8; // 其他的碰撞阈值

  double closest_distance_to_obstacle_model = std::numeric_limits<double>::max();
  double closest_distance_to_obstacle_other = std::numeric_limits<double>::max();
  // 循环遍历障碍物
  for (const auto &obstacle : obstacles)
  {
    // std::cout << "Obstacle Name: " << obstacle.name << std::endl;
    // std::cout << "Obstacle Pose (x, y): (" << obstacle.pose.position.x << ", " << obstacle.pose.position.y << ")" << std::endl;
    double dx = final_pose.position.x - obstacle.pose.position.x;
    double dy = final_pose.position.y - obstacle.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (obstacle.name.find("model") != std::string::npos)
    {
      if (distance < closest_distance_to_obstacle_model)
      {
        closest_distance_to_obstacle_model = distance;
      }
    }
    else
    {
      if (distance < closest_distance_to_obstacle_other)
      {
        closest_distance_to_obstacle_other = distance;
      }
    }
  }

  // 根据最终的距离计算碰撞分数
  if (closest_distance_to_obstacle_model < min_collision_distance_model)
  {
    return 0.0;
  }
  else if (closest_distance_to_obstacle_model <= avoidance_distance_model)
  {
    // 计算model的避障范围内分数
    double score = 1.0 - (closest_distance_to_obstacle_model - min_collision_distance_model) /
                             (avoidance_distance_model - min_collision_distance_model);
    return score;
  }
  // else if (closest_distance_to_obstacle_model <= collision_distance_threshold_model)
  // {
  //   return 0.5;
  // }
  else if (closest_distance_to_obstacle_other < min_collision_distance_other)
  {
    return 0.0;
  }
  else if (closest_distance_to_obstacle_other <= avoidance_distance_other)
  {
    // 计算其他障碍物的避障范围内分数
    double score = 1.0 - (closest_distance_to_obstacle_other - min_collision_distance_other) /
                             (avoidance_distance_other - min_collision_distance_other);
    return score;
  }
  // else if (closest_distance_to_obstacle_other <= collision_distance_threshold_other)
  // {
  //   return 0.5;
  // }
  else
  {
    return 1.0;
  }
}



double DWAPlanner::FindMaxDistance(const std::vector<geometry_msgs::Twist> &twist_vector,
                                   const geometry_msgs::Pose &current_pose)
{
  double max_distance = 1.0;
  // 遍历每组速度角速度，计算对应的距离
  for (const auto &twist : twist_vector)
  {
    geometry_msgs::Pose final_pose = PredictPose(current_pose, twist, time);
    double dx = final_pose.position.x - target_pose.position.x;
    double dy = final_pose.position.y - target_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    // 更新最大距离
    if (distance > max_distance)
    {
      max_distance = distance;
    }
  }
  return max_distance;
}
double DWAPlanner::CalculateDistance(const geometry_msgs::Pose &final_pose)
{
  // 计算位置之间的距离
  double dx = final_pose.position.x - target_pose.position.x;
  double dy = final_pose.position.y - target_pose.position.y;
  double distance_position = std::sqrt(dx * dx + dy * dy);
  // 计算朝向之间的角度差异
  double target_yaw = atan2(2.0 * (target_pose.orientation.z * target_pose.orientation.w +
                                   target_pose.orientation.x * target_pose.orientation.y),
                            1.0 - 2.0 * (target_pose.orientation.y * target_pose.orientation.y +
                                         target_pose.orientation.z * target_pose.orientation.z));
  double final_yaw = atan2(
      2.0 * (final_pose.orientation.z * final_pose.orientation.w + final_pose.orientation.x * final_pose.orientation.y),
      1.0 - 2.0 * (final_pose.orientation.y * final_pose.orientation.y +
                   final_pose.orientation.z * final_pose.orientation.z));
  double yaw_difference = std::abs(target_yaw - final_yaw);
  // 将角度差归一化为 [0, 1]
  double normalized_angle_distance = yaw_difference / 2 * M_PI;
  // 将距离归一化为【0，1】
  // 最大距离引入
  double normalized_distance_distance = distance_position / (max_distance * 1);
  double distance = normalized_distance_distance;
  return distance;
}
double DWAPlanner::CalculateScore(const geometry_msgs::Twist &twist)
{
  // 计算距离目标的距离
  double distance = CalculateDistance(final_pose);
  // 计算碰撞惩罚
  double collision = CalculateCollision(final_pose);
  // if (collision<1)
  // {
  //   distance=-distance;
  // }
  // 设置权重
  double distance_weight = -0.1;
  double collision_weight = 10000;
  // 计算距离分数和碰撞分数
  double distance_score = distance_weight * distance;
  double collision_score = collision_weight * collision;
  // 计算总分数
  double score = collision_score + distance_score;
  // 如果当前分数比最佳分数高，更新最佳分数和对应的 twist 值
  if (score > best_final_score)
  {
    best_final_score = score;
    best_twist = twist;
  }

  return score;
}

// //新的碰撞算法
// bool IsPointInLine(const geometry_msgs::Point& point, const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end) {
//     // 计算点到直线的距离
//     double distance = fabs((line_end.y - line_start.y) * point.x - (line_end.x - line_start.x) * point.y +
//                             line_end.x * line_start.y - line_end.y * line_start.x) /
//                       sqrt(pow(line_end.y - line_start.y, 2) + pow(line_end.x - line_start.x, 2));
//     // 假设阈值为一定距离
//     const double threshold_distance = 0.5;

//     // 如果点到直线的距离小于阈值，认为在直线上
//     return (distance < threshold_distance);
// }

// bool IsPointInCircle(const geometry_msgs::Point& point, const geometry_msgs::Point& circle_center, double circle_radius) {
//     // 计算点到圆心的距离
//     double distance = std::hypot(point.x - circle_center.x, point.y - circle_center.y);

//     // 如果点到圆心的距离小于圆的半径，认为在圆内
//     return (distance < circle_radius);
// }

// double DWAPlanner::CalculateCollision(const geometry_msgs::Pose& final_pose) {
//     double collision_penalty = 0.0;

//     // 示例：遍历所有障碍物，判断最终位置是否与线段或圆相交或过于靠近
//     for (const auto& obstacle : obstacles) {
//         if (obstacle.type == "line") {
//             // 判断最终位置是否在线段内
//             bool is_inside_line = IsPointInLine(final_pose.position, obstacle.start_point, obstacle.end_point);
//             if (is_inside_line) {
//                 collision_penalty += 100.0; // 示例：在线段内增加碰撞惩罚
//             }
//         } else if (obstacle.type == "circle") {
//             // 判断最终位置是否在圆内
//             bool is_inside_circle = IsPointInCircle(final_pose.position, obstacle.center, obstacle.radius);
//             if (is_inside_circle) {
//                 collision_penalty += 100.0; // 示例：在圆内增加碰撞惩罚
//             }
//         }
//     }

//     return collision_penalty;
// }
// bool IsPointInLine(const geometry_msgs::Point& point, const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end) {
//     // 计算点到直线的距离
//     double distance = fabs((line_end.y - line_start.y) * point.x - (line_end.x - line_start.x) * point.y +
//                             line_end.x * line_start.y - line_end.y * line_start.x) /
//                       sqrt(pow(line_end.y - line_start.y, 2) + pow(line_end.x - line_start.x, 2));
//     // 假设阈值为一定距离
//     const double threshold_distance = 0.5;

//     // 如果点到直线的距离小于阈值，认为在直线上
//     return (distance < threshold_distance);
// }

// bool IsPointInCircle(const geometry_msgs::Point& point, const geometry_msgs::Point& circle_center, double circle_radius) {
//     // 计算点到圆心的距离
//     double distance = std::hypot(point.x - circle_center.x, point.y - circle_center.y);

//     // 如果点到圆心的距离小于圆的半径，认为在圆内
//     return (distance < circle_radius);
// }

// double DWAPlanner::CalculateCollision(const geometry_msgs::Pose& final_pose) {
//     double collision_penalty = 0.0;

//     // 示例：遍历所有障碍物，判断最终位置是否与线段或圆相交或过于靠近
//     for (const auto& obstacle : obstacles) {
//         if (obstacle.type == "line") {
//             // 判断最终位置是否在线段内
//             bool is_inside_line = IsPointInLine(final_pose.position, obstacle.start_point, obstacle.end_point);
//             if (is_inside_line) {
//                 collision_penalty += 100.0; // 示例：在线段内增加碰撞惩罚
//             }
//         } else if (obstacle.type == "circle") {
//             // 判断最终位置是否在圆内
//             bool is_inside_circle = IsPointInCircle(final_pose.position, obstacle.center, obstacle.radius);
//             if (is_inside_circle) {
//                 collision_penalty += 100.0; // 示例：在圆内增加碰撞惩罚
//             }
//         }
//     }

//     return collision_penalty;
// }

// double DWAPlanner::CalculateCollision(const geometry_msgs::Pose& final_pose)
// {
//   double collision_distance_threshold = 1.7;  // 设置碰撞距离阈值，
//   for (const geometry_msgs::Pose& obstacle : obstacles)
//   {
//     // 计算机器人和障碍物之间的距离
//     double dx = final_pose.position.x - obstacle.position.x;
//     double dy = final_pose.position.y - obstacle.position.y;

//     double distance1 = std::sqrt(dx * dx + dy * dy);

//     double dx1 = current_pose.position.x - obstacle.position.x;
//     double dy1 = current_pose.position.y - obstacle.position.y;
//       double distance2 = std::sqrt(dx1 * dx1 + dy1 * dy1);
//   if (distance1 <= collision_distance_threshold  )
//     {
//       double score = 1-distance1 / collision_distance_threshold / 2;
//       return score;
//     }
//     // else
//     // {
//     //   double score=1.0;
//     //   return score;
//     // }

//   }
//   // 返回综合得分
//   return 1;
// }