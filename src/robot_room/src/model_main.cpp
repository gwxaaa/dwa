#include "Model.h"
#include <iostream>

int main()
{
    std::string model_name = "wall1.1";

    // 获取指定模型的尺寸信息
    ModelSize model_size = getModelSize(model_name);
    // 输出获取到的模型尺寸信息
    std::cout << "Model Size:" << std::endl;
    std::cout << "Length: " << model_size.length << std::endl;
    std::cout << "Width: " << model_size.width << std::endl;
    std::cout << "Height: " << model_size.height << std::endl;

    // 指定 Z 坐标下长方体一圈外围的点（示例使用默认 Z 坐标值 0.0）
    std::vector<Point> perimeter = calculatePerimeterAtZ({model_size.length, model_size.width, model_size.height}, 0.0);

    // 输出长方体一圈外围的点坐标
    std::cout << "Perimeter Points at Z = 0:" << std::endl;
    for (const auto &point : perimeter)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}