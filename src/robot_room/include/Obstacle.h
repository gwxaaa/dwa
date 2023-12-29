#ifndef OBSTACLE_SETUP_H
#define OBSTACLE_SETUP_H
#pragma once
#include <vector>
struct Point
{
    double x;
    double y;
};

enum FitType
{
    LINE,
    CIRCLE
};

struct Line
{
    Point start;
    Point end;
};

struct Circle
{
    Point center;
    double radius;
};

struct FitResult
{
    FitType type;
    Line lineResult;
    Circle circleResult;
};

class Obstacle
{
public:
    std::vector<FitResult> Obstacle::fitLineOrCircle(const std::vector<Point> &points);

private:
    // FitResult fitLineOrCircle(const std::vector<Point> &points);
    // std::vector<Line> getLinesFromPoints(const std::vector<Point> &points);
    // std::vector<Circle> getCirclesFromPoints(const std::vector<Point> &points);
    double threshold_value;
    double calculateVariance(const std::vector<Point> &points);
    Line fitLine(const std::vector<Point> &points);
    Circle fitCircle(const std::vector<Point> &points);
};

#endif // OBSTACLE_H
