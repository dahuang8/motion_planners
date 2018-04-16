#include <iostream>
#include "gtest/gtest.h"
#include "Tree.h"
#include "Obstacle.h"
#include "CollisionChecker.h"
#include "UniformSampler.h"
#include "ConvexRegion.h"
#include <memory>
#include <stdlib.h>
#include "RRT.h"
#include "SimpleOptimizer.h"
#include <fstream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

QT_CHARTS_USE_NAMESPACE

TEST (TreeTest, TreeConstructionAndTraverse) {
    auto root = new plan::TreeNode();
    root->SetParent(nullptr); // create a root

    for(int i0 = 0; i0 < 10; i0++) {
        //create some subtrees
        plan::TreeNode * subtree = new plan::TreeNode();
        double val[6] = {1,2,3,4,5,6};
        plan::Point pt(val);
        subtree->SetValue(pt);
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
    p[0] << 0.1, 0.1;
    p[1] << 0.2, 0.1;
    p[2] << 0.2, 0.2;
    p[3] << 0.1, 0.2;

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
        plan::Point query_pt; query_pt << tx, ty;
        auto rtn = c_checker.isPointInCollision(query_pt);
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
            plan::Point query_pt; query_pt << tx[i1], ty[i1];
            auto rtn = c_checker.isPointInCollision(query_pt);
            EXPECT_EQ(rtn, false);
        }
    }
}

TEST (SamplerTest, UniformSampler) {
    plan::UniformSampler uni_sampler;
    std::shared_ptr< plan::ConvexRegion > conf_space = std::make_shared< plan:: ConvexRegion > ();
    plan::Point p[4];
    // a square in counter clock wise
    p[0] << 0.1, 0.1;
    p[1] << 0.2, 0.1;
    p[2] << 0.2, 0.2;
    p[3] << 0.1, 0.2;
    for (int i0 = 0; i0 < 4; i0++)
        conf_space->AddContourPoint(p[i0]);
    uni_sampler.SetConfigurationSpace(conf_space);
    std::vector< plan::Point > samples;
    uni_sampler.GenerateRandomSamples(10, samples);
    for ( auto & smpl : samples ) {
        for (int i1 = 0; i1 < smpl.size(); i1++) {
            std::cout << smpl(i1) << ", ";
        }
        std::cout << std::endl;
        EXPECT_EQ(conf_space->isInside(smpl), true);
    }
}

TEST (Planner, RRT_NO_STEPSIZE) {
    // test case for RRT planner. template for others.
    // step 1, prepare the configuration space
    // step 2, prepare the obstacles
    // step 3, query the path

    // configuration space is a 1m by 1m square
    std::shared_ptr< plan::ConvexRegion > conf_space = std::make_shared< plan::ConvexRegion > ();
    plan::Point p[4];
    p[0] << 0, 0;
    p[1] << 1, 0;
    p[2] << 1, 1;
    p[3] << 0, 1;
    for (auto & pt : p)
        conf_space->AddContourPoint(pt);
    // put some box shape obstacles
    std::vector< std::shared_ptr< plan::Obstacle > > obstacles;
    auto obstacle = std::make_shared< plan::Obstacle > ();
    plan::Point pobs[4];
    pobs[0] << 0.4, 0.4;
    pobs[1] << 0.6, 0.4;
    pobs[2] << 0.6, 0.6;
    pobs[3] << 0.4, 0.6;
    for(auto &pt : pobs)
        obstacle->InsertContourPt(pt);
    obstacles.push_back(obstacle);
    // rrt planner
    plan::RRT rrt;
    rrt.SetConfSpace(conf_space);
    rrt.SetObstacles(obstacles);
    plan::Point a, b;
    a << 0.1, 0.2;
    b << 0.9, 0.9;
    std::deque< plan::Point > path_res;
    bool rtn = rrt.FindAPath(a, b, 0.01, path_res);
    EXPECT_EQ(rtn, true);
    std::ofstream outfile("./rrt_test.log");
    for (int i0 = 0; i0 < path_res.size(); i0++) {
        auto &pt = path_res[i0];
        outfile << pt(0) << " " << pt(1) << std::endl;
    }
    outfile.close();
}

TEST (Planner, RRT_STEPSIZE) {
    // test case for RRT planner. template for others.
    // step 1, prepare the configuration space
    // step 2, prepare the obstacles
    // step 3, query the path

    // configuration space is a 1m by 1m square
    std::shared_ptr< plan::ConvexRegion > conf_space = std::make_shared< plan::ConvexRegion > ();
    plan::Point p[4];
    p[0] << 0, 0;
    p[1] << 1, 0;
    p[2] << 1, 1;
    p[3] << 0, 1;
    for (auto & pt : p)
        conf_space->AddContourPoint(pt);
    // put some box shape obstacles
    std::vector< std::shared_ptr< plan::Obstacle > > obstacles;
    auto obstacle = std::make_shared< plan::Obstacle > ();
    plan::Point pobs[4];
    pobs[0] << 0.4, 0.4;
    pobs[1] << 0.6, 0.4;
    pobs[2] << 0.6, 0.6;
    pobs[3] << 0.4, 0.6;
    for(auto &pt : pobs)
        obstacle->InsertContourPt(pt);
    obstacles.push_back(obstacle);
    // rrt planner
    plan::RRT rrt;
    rrt.SetConfSpace(conf_space);
    rrt.SetObstacles(obstacles);
    rrt.SetStepSize(0.05);
    plan::Point a, b;
    a << 0.1, 0.2;
    b << 0.9, 0.9;
    std::deque< plan::Point > path_res;
    bool rtn = rrt.FindAPath(a, b, 0.01, path_res);
    EXPECT_EQ(rtn, true);
    std::ofstream outfile("./rrt_test2.log");
    for (int i0 = 0; i0 < path_res.size(); i0++) {
        auto &pt = path_res[i0];
        outfile << pt(0) << " " << pt(1) << std::endl;
    }
    outfile.close();

    plan::SimpleOptimizer optimizer(obstacles);
    std::deque< plan::Point > optimized_path;
    optimizer.OptimizePath(path_res, optimized_path);

    std::ofstream outfile2("./rrt_test3.log");
    for (int i0 = 0; i0 < optimized_path.size(); i0++) {
        auto &pt = optimized_path[i0];
        outfile2 << pt(0) << " " << pt(1) << std::endl;
    }
    outfile2.close();

}

/*
TEST (MATPLOTLIB, BASICPLOT) {
    int argc = 0;
    QApplication a(argc, NULL);

    QLineSeries *series = new QLineSeries();

    series->append(0, 6);
    series->append(2, 4);
    series->append(3, 8);
    series->append(7, 4);
    series->append(10, 5);
    *series << QPointF(11, 1) << QPointF(13, 3) << QPointF(17, 6) << QPointF(18, 3) << QPointF(20, 2);

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("Simple line chart example");

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(400, 300);
    window.show();

    a.exec(); // The window won't stop.
}*/
