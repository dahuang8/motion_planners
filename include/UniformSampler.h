//
// Created by kevinhuang on 4/12/18.
//

#ifndef MOTIONPLAN_UNIFORMSAMPLER_H
#define MOTIONPLAN_UNIFORMSAMPLER_H

#include "Sampler.h"

namespace plan {
    class UniformSampler : public Sampler {
    public:
        UniformSampler();
        ~UniformSampler();
        void GenerateRandomSamples(uint16_t num_samples, std::vector< Point > & samples);
    private:
        Point GenerateOneRandomSample(const std::vector<double> &mins, const std::vector<double> &maxes);
    };
}


#endif //MOTIONPLAN_UNIFORMSAMPLER_H
