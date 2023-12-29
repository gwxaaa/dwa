#include "Obstacle.h"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

// 根据方差判断拟合直线还是圆
//在信息设定为五个点的时候进行分段，然后将模型的信息判断是直线还是圆形进行拟合，将拟合的结果返回，就可以得到不同模型的分类。
std::vector<FitResult> Obstacle::fitLineOrCircle(const std::vector<Point>& points) {
    std::vector<std::vector<Point>> dividedPoints; // 存储分割后的子集
    // 将数据点分割为多个子集，这里假设每个子集的大小为 segmentSize
    const size_t segmentSize = 5;
    size_t numSegments = points.size() / segmentSize;
    for (size_t i = 0; i < numSegments; ++i) {
        std::vector<Point> segment(points.begin() + i * segmentSize, points.begin() + (i + 1) * segmentSize);
        dividedPoints.push_back(segment);
    }
    // 对每个子集进行拟合
    std::vector<FitResult> results;
    for (const auto& segment : dividedPoints) {
        double variance = calculateVariance(segment);
        FitResult fitResult;
        if (variance < threshold_value) {
            fitResult.type = CIRCLE;
            fitResult.circleResult = fitCircle(segment);
        } else {
            fitResult.type = LINE;
            fitResult.lineResult = fitLine(segment);
        }
        results.push_back(fitResult);
    }
    // 返回拟合的结果
    return results;
}
// 计算数据点的方差,判断应该拟合成什么
double Obstacle::calculateVariance(const std::vector<Point> &points)
{
    double sumX = 0.0, sumY = 0.0, sumXX = 0.0, sumYY = 0.0, sumXY = 0.0;
    for (const auto &point : points)
    {
        sumX += point.x;
        sumY += point.y;
        sumXX += point.x * point.x;
        sumYY += point.y * point.y;
        sumXY += point.x * point.y;
    }
    double n = static_cast<double>(points.size());
    double variance = (sumXX - sumX * sumX / n) * (sumYY - sumY * sumY / n) - (sumXY - sumX * sumY / n) * (sumXY - sumX * sumY / n);
    return variance;
}

// 使用最小二乘法拟合直线
Line Obstacle::fitLine(const std::vector<Point> &points)
{
    Eigen::MatrixXd A(points.size(), 2);
    Eigen::VectorXd b(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = 1;
        b(i) = points[i].y;
    }

    Eigen::VectorXd result = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    Line line;
    line.start.x = points[0].x;
    line.start.y = result(0) * points[0].x + result(1);
    line.end.x = points[points.size() - 1].x;
    line.end.y = result(0) * points[points.size() - 1].x + result(1);
    return line;
}

// 拟合圆
Circle Obstacle::fitCircle(const std::vector<Point> &points)
{
    double sumX = 0, sumY = 0;
    for (const auto &point : points)
    {
        sumX += point.x;
        sumY += point.y;
    }
    Point center;
    center.x = sumX / points.size();
    center.y = sumY / points.size();

    double sumDistance = 0;
    for (const auto &point : points)
    {
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        sumDistance += std::sqrt(dx * dx + dy * dy);
    }
    double radius = sumDistance / points.size();
    Circle circle;
    circle.center = center;
    circle.radius = radius;
    return circle;
}


// FitResult Obstacle::fitLineOrCircle(const std::vector<Point>& points) {
//     std::vector<std::vector<Point>> dividedPoints; // 存储分割后的子集
//     // 将数据点分割为多个子集，这里假设每个子集的大小为 segmentSize
//     const size_t segmentSize = 10;
//     size_t numSegments = points.size() / segmentSize;
//     for (size_t i = 0; i < numSegments; ++i) {
//         std::vector<Point> segment(points.begin() + i * segmentSize, points.begin() + (i + 1) * segmentSize);
//         dividedPoints.push_back(segment);
//     }

//     // 对每个子集进行拟合
//     std::vector<FitResult> results;
//     for (const auto& segment : dividedPoints) {
//         double variance = calculateVariance(segment);
//         FitResult fitResult;
//         if (variance < threshold_value) {
//             fitResult.type = CIRCLE;
//             fitResult.circleResult = fitCircle(segment);
//         } else {
//             fitResult.type = LINE;
//             fitResult.lineResult = fitLine(segment);
//         }

//         results.push_back(fitResult);
//     }

// }

// std::vector<Line> Obstacle::getLinesFromPoints(const std::vector<Point> &points)
// {
//     std::vector<Line> lines;
//     // 分段拟合直线
//     const size_t segmentSize = 5; // 每段的点数
//     size_t numSegments = points.size() / segmentSize;

//     for (size_t i = 0; i < numSegments; ++i)
//     {
//         std::vector<Point> segment(points.begin() + i * segmentSize, points.begin() + (i + 1) * segmentSize);
//         Line line = fitLine(segment); // 使用 fitLine 函数拟合线段
//         lines.push_back(line);
//     }
//     return lines;
// }
// std::vector<Circle> Obstacle::getCirclesFromPoints(const std::vector<Point> &points)
// {
//     std::vector<Circle> circles;
//     const size_t segmentSize = 5; // 每段的点数
//     size_t numSegments = points.size() / segmentSize;
//     Circle circle = fitCircle(points); // 使用 fitCircle 函数拟合整个点集
//     circles.push_back(circle);
//     return circles;
// }