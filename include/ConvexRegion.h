//
// Created by kevinhuang on 4/12/18.
//

#ifndef MOTIONPLAN_CONVEXREGION_H
#define MOTIONPLAN_CONVEXREGION_H

#include <cassert>
#include <vector>
#include "CommonDef.h"
#include "Point.h"
// #include <Eigen/Dense>
#include <algorithm>

namespace plan {
    class ConvexRegion {
    public:
        ConvexRegion () {}
        ~ConvexRegion () {}
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
        void AddContourPoint(Point pt) {
            contour_pts_.push_back(pt);
            // initialization
            if (min_.size() == 0 || max_.size() == 0) {
                for(int i0 = 0; i0 < pt.size(); i0++) {
                    min_.push_back(pt(i0));
                }
                max_ = min_;
            } else {
                assert(min_.size() == max_.size());
                for (int i0 = 0; i0 < min_.size(); i0++) {
                    min_[i0] = std::min(min_[i0], pt(i0));
                    max_[i0] = std::max(max_[i0], pt(i0));
                }
            }
        }
        void ClearContourPoint() {
            contour_pts_.clear();
        }
        void GetMinMax(std::vector<double> &min, std::vector<double> &max) {
            min = min_;
            max = max_;
        }
    private:
        std::vector<plan::Point> contour_pts_; // points need to be counter clockwise
        std::vector<double> min_;
        std::vector<double> max_;
    };
}


#endif //MOTIONPLAN_CONVEXREGION_H
