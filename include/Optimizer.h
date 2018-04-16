//
// Created by kevinhuang on 4/15/18.
//

#ifndef MOTIONPLAN_OPTIMIZER_H
#define MOTIONPLAN_OPTIMIZER_H

#include <vector>
#include <memory>
#include <deque>

#include "Point.h"
#include "Obstacle.h"
#include "ConvexRegion.h"
#include "CollisionChecker.h"

namespace plan {
    class Optimizer {
    public:
        Optimizer() {}
        Optimizer(const std::vector< std::shared_ptr< Obstacle > > &obstacles) {
            SetObstacles(obstacles);
        }
        virtual void OptimizePath(const std::deque< Point > &in_path, std::deque< Point > &out_path) = 0;
        void SetObstacles(const std::vector< std::shared_ptr< Obstacle > > &obstacles) {
            obstacles_.assign(obstacles.begin(), obstacles.end());
            collision_checker_ = std::make_shared<CollisionChecker>();
            for(auto obs : obstacles) {
                collision_checker_->AddObstacle(obs);
            }
        }
    protected:
        std::vector< std::shared_ptr< Obstacle > > obstacles_;
        std::shared_ptr< CollisionChecker > collision_checker_;
    };
}
#endif //MOTIONPLAN_OPTIMIZER_H
