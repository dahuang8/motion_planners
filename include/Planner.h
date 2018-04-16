//
// Created by kevinhuang on 4/10/18.
//

#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include "Point.h"
#include <vector>
#include <deque>

namespace plan {
    // parent class to all motion planners
    class Planner {
    public:
        Planner() {}
        ~Planner() {}
        // success or failure.
        virtual bool FindAPath(Point start, Point end, double tol_radius, std::deque< Point > &path_res) = 0; // child class shall determine if smooth and optimize path are needed
    };
}



#endif //RRT_PLANNER_H
