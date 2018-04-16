//
// Created by dahuang8 on 04/09/2018.
//

#ifndef RRT_SAMPLER_H
#define RRT_SAMPLER_H

#include <cstdint>
#include <vector>
#include <memory>
#include "Point.h"
#include "ConvexRegion.h"

namespace plan {
class Sampler {
public:
    Sampler() {}
    ~Sampler() {}
    void SetConfigurationSpace(std::shared_ptr< ConvexRegion > space) {
        space_ = space;
    }
    virtual void GenerateRandomSamples(uint16_t num_samples, std::vector< Point > & samples) = 0;

protected:
    std::shared_ptr< ConvexRegion > space_;
};
}

#endif //RRT_SAMPLER_H
