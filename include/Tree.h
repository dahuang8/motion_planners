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

namespace plan {

class TreeNode {
public:
    TreeNode() {
        memset(value_, 0, sizeof(value_));
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
        subtrees_.push_back(tree_node);

    }
    void SetValue(const double *p_val, int n) {
        assert(n <= 6);
        for(int i0 = 0; i0 < n; i0++) {
            value_[i0] = p_val[i0];
        }
    }
    void TraverseTree_DepthFirst() { // this is depth-first
        std::cout << "-------------------------------" << std::endl;
        for (int i1 = 0; i1 < 6; i1++) {
            std::cout << this->value_[i1] << " ";
        }
        std::cout << "\n--------------------------------" << std::endl;
        for(int i0 = 0; i0 < subtrees_.size(); i0++) {
            auto subtree = subtrees_.at(i0);
            // print the node's value
            std::cout << "-------------------------------" << std::endl;
            for (int i1 = 0; i1 < 6; i1++) {
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
                    for (int i2 = 0; i2 < 6; i2++) {
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
private:
    double value_[6];
    TreeNode * parent_; // nullptr is for the root
    std::vector< TreeNode * > subtrees_; // a set of children connect to this tree/subtree
private:
    std::vector< TreeNode * > GetChildrenVector() {
        return subtrees_;
    }
};

}

#endif //RRT_TREE_H
