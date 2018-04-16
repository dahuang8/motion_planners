//
// Created by kevinhuang on 4/15/18.
//

#include "SimpleOptimizer.h"

namespace plan {
// simply use line segment to simplify the path
    void SimpleOptimizer::OptimizePath(const std::deque<Point> &in_path, std::deque<Point> &out_path) {
        int ia = 0, ib = 1;
        out_path.push_back(in_path[0]);
        while (ia < in_path.size() - 1 && ib < in_path.size() - 1) {
            bool is_collide = collision_checker_->isEdgeInCollision(in_path[ia], in_path[ib]);
            if (is_collide) {
                if (ib - ia <= 1) {
                    std::cerr << "Input path is invalid." << std::endl;
                    break;
                }
                ia = ib-1;
                out_path.push_back(in_path[ia]);
            } else {
                ib++;
            }
        }
        out_path.push_back(in_path[ib]);
    }
}