//
// Created by kevinhuang on 4/10/18.
//

#ifndef RRT_OBSTACLE_H
#define RRT_OBSTACLE_H

#include <vector>
#include <cassert>
#include <math.h>

namespace plan{
const double epsilon = 1e-4;

class Point { // defined in 2D Cartesian space, if used by robot arm, need forward kinematics to convert into Cartesian space
public:
    Point() {}
    Point(double x, double y) : x_(x), y_(y) {};
    double x_;
    double y_;
    Point operator+(const Point &rhs) {
        Point c;
        c.x_ = x_ + rhs.x_;
        c.y_ = y_ + rhs.y_;
        return c;
    }
    Point operator-(const Point &rhs) {
        Point c;
        c.x_ = x_ - rhs.x_;
        c.y_ = y_ - rhs.y_;
        return c;
    }
    double operator*(const Point &rhs) {return (x_ * rhs.x_ + y_ * rhs.y_);
    }
    Point operator*(double r) {
        Point c;
        c.x_ = x_ * r;
        c.y_ = y_ * r;
        return c;
    }
    double Norm() {
        return sqrt(x_*x_ + y_*y_);
    }
    Point operator/(double d) {
        Point c;
        c.x_ = x_ / d;
        c.y_ = y_ / d;
        return c;
    }
    Point Normalize() {
        double norm = sqrt(x_*x_ + y_*y_);
        Point c(x_/norm, y_/norm);
        return c;
    }
};

class Obstacle { // convex hull type obstacle
public:
    std::vector<plan::Point> contour_pts_; // points need to be counter clockwise
    void InsertContourPt(plan::Point pt) {
        contour_pts_.push_back(pt);
    };
    void ClearContourPt() {
        contour_pts_.clear();
    }
    bool isInside(plan::Point query_pt) {
        // this version deals with 2D obstacle. easy to extend to 3D using similar idea
        // https://www.codeproject.com/Articles/1065730/Point-Inside-Convex-Polygon-in-Cplusplus
        // as we require the contour points are in counter clockwise, the inside normal is on the left
        assert(contour_pts_.size() >= 3);
        for(int i0 = 0; i0 < contour_pts_.size(); i0++) {
            auto a = contour_pts_[i0];
            auto b = contour_pts_[(i0+1) % contour_pts_.size()];
            auto c = b - a;
            Point d(-c.y_, c.x_); // normal towards the inside
            Point e = query_pt - ((a+b) / 2);
            double inner = d * e;
            if (inner < -epsilon)
                return false;
        }
        return true;
    }
};

}

#endif //RRT_OBSTACLE_H
