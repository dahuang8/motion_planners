//
// Created by kevinhuang on 4/10/18.
//

#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

namespace plan {
    // parent class to all motion planners
    class Planner {
    public:
        Planner();
        virtual ~Planner() = 0; // sub class has to implement their own deconstructor
    };
}



#endif //RRT_PLANNER_H
