//
// Created by kevinhuang on 4/10/18.
//

#ifndef RRT_TREE_H
#define RRT_TREE_H

#include <vector>
#include <assert.h>
#include <memory>
#include <cstring>
#include <iostream>
#include "Point.h"

namespace plan {

class TreeNode {
public:
    TreeNode() {
        parent_ = nullptr;
        value_ = Eigen::MatrixXd::Constant(value_.rows(), value_.cols(), 0.0);
    };
    ~TreeNode() {
        for (int i0 = 0; i0 < subtrees_.size(); i0++) {
            TreeNode * node = subtrees_.at(i0);
            delete node;
        }
    };
    void SetParent(TreeNode * tree_node) {
        parent_ = tree_node;
    }
    void AddChild(TreeNode * tree_node) {
        tree_node->SetParent(this);
        subtrees_.push_back(tree_node);
    }
    void SetValue(const Point &pt) {
        value_ = pt;
    }

    double FindClosestPoint(const Point &query_pt, TreeNode **closest_node) {
        double min_dist = (query_pt - value_).norm();
        *closest_node = this;
        for(int i0 = 0; i0 < subtrees_.size(); i0++) {
            auto subtree = subtrees_.at(i0);
            TreeNode *sub_cloest_node;
            double sub_min_dist = subtree->FindClosestPoint(query_pt, &sub_cloest_node);
            if (sub_min_dist < min_dist) {
                *closest_node = sub_cloest_node;
                min_dist = sub_min_dist;
            }
        }
    }
    void TraverseTree_DepthFirst() { // this is depth-first
        std::cout << "-------------------------------" << std::endl;
        for (int i1 = 0; i1 < value_.size(); i1++) {
            std::cout << this->value_[i1] << " ";
        }
        std::cout << "\n--------------------------------" << std::endl;
        for(int i0 = 0; i0 < subtrees_.size(); i0++) {
            auto subtree = subtrees_.at(i0);
            // print the node's value
            std::cout << "-------------------------------" << std::endl;
            for (int i1 = 0; i1 < value_.size(); i1++) {
                std::cout << subtree->value_[i1] << " ";
            }
            std::cout << "\n--------------------------------" << std::endl;
            subtree->TraverseTree_DepthFirst();
        }
    }
    void TraverseTree_BreadthFirst() { // this is depth-first
        // consider self as the root (start point of the traverse)
        std::vector< std::vector< TreeNode * > > all_sub_nodes;
        std::vector< TreeNode * > self_treenode;
        self_treenode.push_back(this);
        all_sub_nodes.push_back(self_treenode);

        while(!all_sub_nodes.empty()) {
            std::vector< std::vector< TreeNode * > > tmp_all_sub_nodes;
            for(int i0 = 0; i0 < all_sub_nodes.size(); i0++) {
                auto subtree = all_sub_nodes.at(i0);
                for (int i1 = 0; i1 < subtree.size(); i1++) {
                    auto node = subtree.at(i1);
                    // print the node's value
                    std::cout << "-------------------------------" << std::endl;
                    for (int i2 = 0; i2 < value_.size(); i2++) {
                        std::cout << node->value_[i2] << " ";
                    }
                    std::cout << "\n--------------------------------" << std::endl;
                    tmp_all_sub_nodes.push_back(node->GetChildrenVector());
                }
            }
            all_sub_nodes.clear();
            all_sub_nodes = tmp_all_sub_nodes;
        }
    }
    Point GetPoint() {
        return value_;
    }
    TreeNode* GetParent() {
        return parent_;
    }
private:
    Point value_;
    TreeNode * parent_; // nullptr is for the root
    std::vector< TreeNode * > subtrees_; // a set of children connect to this tree/subtree
private:
    std::vector< TreeNode * > GetChildrenVector() {
        return subtrees_;
    }
};

}

#endif //RRT_TREE_H
