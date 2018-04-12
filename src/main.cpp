#include <iostream>

#include "gtest/gtest.h"
#include "Tree.h"
#include "Obstacle.h"
#include "CollisionChecker.h"
#include <memory>
#include <stdlib.h>

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

TEST (CollisionCheckTest, PointCollisionCheck) {
    plan::CollisionChecker c_checker;
    auto o0 = std::make_shared< plan::Obstacle > ();
    plan::Point p[4];
    // a square in counter clock wise
    p[0].x_ = 0.1; p[0].y_ = 0.1;
    p[1].x_ = 0.2; p[1].y_ = 0.1;
    p[2].x_ = 0.2; p[2].y_ = 0.2;
    p[3].x_ = 0.1; p[3].y_ = 0.2;
    for (auto pt : p) {
        o0->InsertContourPt(pt);
    }
    c_checker.AddObstacle(o0);

    // randomly generate 100 points inside the square
    for (int i0 = 0; i0 < 100; i0++) {
        double rx = (double)rand() / (double) RAND_MAX / 10.0;
        double ry = (double)rand() / (double) RAND_MAX / 10.0;
        auto tx = 0.1 + rx;
        auto ty = 0.1 + ry;
        auto rtn = c_checker.isPointInCollision(plan::Point(tx, ty));
        EXPECT_EQ(rtn, true);
    }
    // randomly generate 400 points outside the square
    for (int i0 = 0; i0 < 100; i0++) {
        double rx = (double)rand() / (double) RAND_MAX / 10.0;
        double ry = (double)rand() / (double) RAND_MAX / 10.0;
        double tx[4], ty[4];
        tx[0] = 0.1 - rx; ty[0] = ry - 0.5;
        tx[1] = 0.2 + rx; ty[1] = ry - 0.5;
        tx[2] = rx - 0.5; ty[2] = 0.1 - ry;
        tx[3] = rx - 0.5; ty[3] = 0.2 + ry;
        for (int i1 = 0; i1 < 4; i1++) {
            auto rtn = c_checker.isPointInCollision(plan::Point(tx[i1], ty[i1]));
            EXPECT_EQ(rtn, false);
        }
    }
}