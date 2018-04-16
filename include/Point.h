//
// Created by kevinhuang on 4/12/18.
//

#ifndef MOTIONPLAN_POINT_H
#define MOTIONPLAN_POINT_H
#include <math.h>
#include <Eigen/Dense>
#include "CommonDef.h"
namespace plan {
    typedef Eigen::Matrix< double, SPACE_DOF, 1 > Point;
}
#endif //MOTIONPLAN_POINT_H
