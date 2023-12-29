#include "BoxCalculate.h"
#include <cmath>

std::vector<Point> calculatePerimeterAtZ(const Box& box, double zCoordinate) {
    double halfLength = box.length / 2.0;
    double halfWidth = box.width / 2.0;
    double cosYaw = cos(box.yaw);
    double sinYaw = sin(box.yaw);
    std::vector<Point> perimeterPoints;

    for (double angle = 0; angle < 2 * M_PI; angle += 0.1) {
        double x = halfLength * cos(angle);
        double y = halfWidth * sin(angle);

        double rotatedX = x * cosYaw - y * sinYaw + box.center.x;
        double rotatedY = x * sinYaw + y * cosYaw + box.center.y;

        perimeterPoints.push_back({rotatedX, rotatedY, zCoordinate});
    }

    return perimeterPoints;
}