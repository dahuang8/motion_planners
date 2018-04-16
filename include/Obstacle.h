//
// Created by kevinhuang on 4/10/18.
//

#ifndef RRT_OBSTACLE_H
#define RRT_OBSTACLE_H

#include <vector>
#include <cassert>
#include "CommonDef.h"
#include <math.h>
#include "Point.h"

namespace plan{
class Obstacle { // convex hull type obstacle
public:
    std::vector<Point> contour_pts_; // points need to be counter clockwise
    void InsertContourPt(plan::Point pt) {
        contour_pts_.push_back(pt);
    };
    void ClearContourPt() {
        contour_pts_.clear();
    }
    bool isInside(Point query_pt) {
        // this version deals with 2D obstacle. easy to extend to 3D using similar idea
        // https://www.codeproject.com/Articles/1065730/Point-Inside-Convex-Polygon-in-Cplusplus
        // as we require the contour points are in counter clockwise, the inside normal is on the left
        assert(contour_pts_.size() >= 3);
        for(int i0 = 0; i0 < contour_pts_.size(); i0++) {
            auto a = contour_pts_[i0];
            auto b = contour_pts_[(i0+1) % contour_pts_.size()];
            auto c = b - a;
            Point d;
            d << -c(1), c(0); // normal towards the inside
            Point e = query_pt - ((a+b) / 2);
            double inner = d.dot(e);
            if (inner < -epsilon)
                return false;
        }
        return true;
    }
};

}

#endif //RRT_OBSTACLE_H
