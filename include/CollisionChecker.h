//
// Created by dahuang8 on 04/09/2018.
//

#ifndef RRT_COLLISIONCHECKER_H
#define RRT_COLLISIONCHECKER_H

#include <memory>
#include <vector>

#include "Tree.h"
#include "Obstacle.h"

namespace plan {
    class CollisionChecker {
    public:
        CollisionChecker() {}
        ~CollisionChecker() {}
        bool isEdgeInCollision(Point a, Point b) {
            bool rtn = false;
            // sample the edge and call isPointInCollision for each sample
            double step = 0.01; // 10mm step
            Point c = (b - a).Normalize();
            double distance = (b-a).Norm();
            int steps = (distance + step) / step;
            for(int i0 = 0; i0 < steps; i0++) {
                Point d;
                if (i0 == (steps-1)) {
                    d = b;
                } else if (i0 == 0) {
                    d = a;
                } else {
                    d = d + c*step;
                }
                rtn = isPointInCollision(d);
                if (rtn)
                    break;
            }
            return rtn;
        }
        bool isPointInCollision(Point a) {
            bool rtn = false;
            for (auto obs : obs_) {
                rtn = obs->isInside(a);
                if (rtn)
                    break;
            }
            return rtn;
        }
        void AddObstacle(std::shared_ptr< Obstacle > p_ob) {
            obs_.push_back(p_ob);
        }
        void ClearObstacle() {
            obs_.clear();
        }
    private:

        std::vector< std::shared_ptr< Obstacle > > obs_;
    };
}

#endif //RRT_COLLISIONCHECKER_H
