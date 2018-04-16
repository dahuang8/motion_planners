//
// Created by dahuang8 on 04/09/2018.
//

#include <UniformSampler.h>
#include "RRT.h"

namespace plan {
    bool RRT::FindAPath(Point start, Point end, double tol_radius, std::deque<Point> &path_res) {
        // Algorithm: classic RRT try to connect a random valid sample to the closest node each time, stop util reach
        // the target zone.
        assert(this->conf_space_ != nullptr); // conf_space_ has to be set prior to this call
        assert(this->collision_checker_ != nullptr);
        // construct the tree's root
        root_ = std::make_shared< TreeNode >();
        root_->SetValue(start);
        last_path_.clear();

        int max_iterations = 50000;
        int max_sample_trials = 1000;

        UniformSampler uni_sampler;
        uni_sampler.SetConfigurationSpace(conf_space_);

        TreeNode *target;
        bool is_path_found = false;
        double dist_to_target = (start - end).norm();
        do {
            bool is_sample_valid = false;
            Point smpl;
            TreeNode *res_node;
            for (int i0 = 0; i0 < max_sample_trials; i0++) {
                // Random sample a point in configuration sapce
                std::vector< Point > smpls;
                uni_sampler.GenerateRandomSamples(1, smpls);
                // since we only ask for one sample
                smpl = smpls[0];
                root_->FindClosestPoint(smpl, &res_node);
                // Only move step_size_
                if (step_size_ != 0) {
                    auto dist_vec = smpl - res_node->GetPoint();
                    smpl = dist_vec.normalized() * step_size_ + res_node->GetPoint();
                }
                // Check if the line between smpl and res_node is collision-free
                if ( !(collision_checker_->isEdgeInCollision(smpl, res_node->GetPoint())) ) {
                    is_sample_valid = true;
                    break;
                }
            }
            // connect to that node
            if (is_sample_valid) {
                TreeNode *new_node = new TreeNode(); // resource is managed by root_
                new_node->SetValue(smpl);
                res_node->AddChild(new_node);
                // update dist_to_target
                dist_to_target = (smpl-end).norm();
                if (dist_to_target <= tol_radius) {
                    target = new_node;
                    is_path_found = true;
                }
            } else {
                std::cerr << "Fail to find a valid random sample within " << max_sample_trials << " trials." << std::endl;
            }
        } while (!is_path_found && (max_iterations--) > 0);

        if (is_path_found) {
            // construct the path. from end to start
            TreeNode * tmp_node = target;
            while (tmp_node->GetParent() != nullptr) {
                last_path_.push_front(tmp_node);
                path_res.push_front(tmp_node->GetPoint());
                tmp_node = tmp_node->GetParent();
            }
            last_path_.push_front(tmp_node);
            path_res.push_front(tmp_node->GetPoint());
        } else {
            std::cerr << "Fail to find a valid path" << std::endl;
        }
        return is_path_found;
    }
}
