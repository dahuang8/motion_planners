//
// Created by kevinhuang on 4/12/18.
//

#include "UniformSampler.h"
#include <time.h>
#include "Point.h"
#include <cassert>

namespace plan {
    UniformSampler::UniformSampler() {
        space_ = nullptr;
        srand(time(NULL));
    }
    UniformSampler::~UniformSampler() {}
    void UniformSampler::GenerateRandomSamples(uint16_t num_samples, std::vector< Point > & samples) {
        samples.clear();
        // make sure convex region has been set
        if (space_ != nullptr) {
            std::vector< double > mins, maxes;
            space_->GetMinMax(mins, maxes);
            for (int i0 = 0; i0 < num_samples; i0++)
                samples.push_back(GenerateOneRandomSample(mins, maxes));
        }
    }
    Point UniformSampler::GenerateOneRandomSample(const std::vector<double> &mins,
                                                                  const std::vector<double> &maxes) {
        assert(mins.size() == maxes.size());
        Point query_pt;
        assert(mins.size() == query_pt.size());
        query_pt = Eigen::MatrixXd::Constant(query_pt.rows(), query_pt.cols(), 0.0);
        // generate random point that is inside the region
        do {
            for(int i0 = 0; i0 < mins.size(); i0++) {
                double rval = ((double) rand() / (double)RAND_MAX) * (maxes[i0]-mins[i0]) + mins[i0]; // uniform sampling
                query_pt(i0) = rval;
            }
        } while (!space_->isInside(query_pt));
        return query_pt;
    }
}