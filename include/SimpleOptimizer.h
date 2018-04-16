//
// Created by kevinhuang on 4/15/18.
//

#ifndef MOTIONPLAN_SIMPLEOPTIMIZER_H
#define MOTIONPLAN_SIMPLEOPTIMIZER_H

#include "Optimizer.h"

namespace plan {
    class SimpleOptimizer : public Optimizer {
    public:
        SimpleOptimizer(const std::vector< std::shared_ptr< Obstacle > > &obstacles) : Optimizer(obstacles) { }
        void OptimizePath(const std::deque< Point > &in_path, std::deque< Point > &out_path);
    };
}


#endif //MOTIONPLAN_SIMPLEOPTIMIZER_H
