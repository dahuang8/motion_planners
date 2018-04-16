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

bool TestRRTNoStepSize(std::deque< plan::Point > &path_res) {
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
    return rrt.FindAPath(a, b, 0.01, path_res);
}

bool TestRRTWithStepSizeOptimizer(std::deque< plan::Point > &raw_path, std::deque< plan::Point > &optimal_path) {
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
    bool rtn = rrt.FindAPath(a, b, 0.01, raw_path);
    plan::SimpleOptimizer optimizer(obstacles);
    optimizer.OptimizePath(raw_path, optimal_path);
    return rtn;
}

int main(int argc, char ** argv) {
    QApplication a(argc, argv);
    QLineSeries *series0 = new QLineSeries();
    QLineSeries *series1 = new QLineSeries();
    QLineSeries *series2 = new QLineSeries();

    std::deque< plan::Point > path_res;
    TestRRTNoStepSize(path_res);
    for (auto &pt : path_res) {
        series0->append(pt(0), pt(1));
    }
    series0->setName("Path w/o Step Size");
    path_res.clear();
    std::deque< plan::Point > raw_path, optimal_path;
    TestRRTWithStepSizeOptimizer(raw_path, optimal_path);
    for (auto &pt : raw_path) {
        series1->append(pt(0), pt(1));
    }
    series1->setName("Path w/ Step Size");
    for (auto &pt : optimal_path) {
        series2->append(pt(0), pt(1));
    }
    series2->setName("Path w/ Optimization");
    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(series0);
    chart->addSeries(series1);
    chart->addSeries(series2);
    chart->createDefaultAxes();
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->setTitle("RRT Motion Planning Example");

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(400, 300);
    window.show();

    return a.exec(); // The window won't stop.
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
