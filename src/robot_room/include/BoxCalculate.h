#ifndef BOX_CALCULATOR_H
#define BOX_CALCULATOR_H

#include <vector>

struct Point {
    double x, y, z;
};

struct Box {
    Point center;
    double length, width, height;
    double yaw;
};

std::vector<Point> calculatePerimeterAtZ(const Box& box, double zCoordinate);

#endif // BOX_CALCULATOR_H

