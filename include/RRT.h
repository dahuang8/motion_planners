//
// Created by dahuang8 on 04/09/2018.
//

#ifndef RRT_RRT_H
#define RRT_RRT_H

#include <deque>
#include "Planner.h"
#include "Tree.h"
#include "Obstacle.h"
#include "ConvexRegion.h"
#include "CollisionChecker.h"

namespace plan {
    class RRT : public Planner {
    public:
        RRT() : conf_space_(nullptr), root_(nullptr), collision_checker_(nullptr), step_size_(0) {}
        ~RRT() {}
        bool FindAPath(Point start, Point end, double tol_radius, std::deque< Point > &path_res); // Init the root_ therein
        void SetConfSpace(std::shared_ptr< ConvexRegion > space) {
            conf_space_ = space;
        }
        void SetObstacles(std::vector< std::shared_ptr< Obstacle > > obstacles) {
            for(auto obstacle : obstacles)
                obstacles_.push_back(obstacle);
            // init collision checker automatically
            collision_checker_ = std::make_shared< CollisionChecker > (); // old checker will be released
            for(auto obstacle: obstacles_)
                collision_checker_->AddObstacle(obstacle);
        }
        void SetStepSize(double step_size) {
            step_size_ = step_size;
        }
    private:
        bool SmoothPath();
        std::shared_ptr< TreeNode > root_;
        std::deque< TreeNode * > last_path_;
        std::shared_ptr< ConvexRegion > conf_space_; // use convexregion to define a configuration space. if less or equal to 3DoF (workspace), it shall work pretty efficiently. for higher DoF, could use a simpler region representation.
        std::vector< std::shared_ptr< Obstacle > > obstacles_;
        std::shared_ptr< CollisionChecker > collision_checker_;
        double step_size_; // distance of each step, 0 means no limitation
    };
}

#endif //RRT_RRT_H
