#include <iostream>

#include "gtest/gtest.h"
#include "Tree.h"
#include "Obstacle.h"
#include <memory>

TEST (TreeTest, TreeConstructionAndTraverse) {
    auto root = new plan::TreeNode();
    root->SetParent(nullptr); // create a root

    for(int i0 = 0; i0 < 10; i0++) {
        //create some subtrees
        plan::TreeNode * subtree = new plan::TreeNode();
        double val[6] = {1,2,3,4,5,6};
        subtree->SetValue(val, 6);
        subtree->SetParent(root);
        root->AddChild(subtree);
    }
    // root->TraverseTree_BreadthFirst();
    root->TraverseTree_DepthFirst();
    delete root;
    root = nullptr;
}

TEST (CollisionCheckTest, Point) {
    
}