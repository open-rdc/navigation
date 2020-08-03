#ifndef EXPANSION_RESETTING_H
#define EXPANSION_RESETTING_H

#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_laser.h"
#include <random>

namespace amcl{
    class  AMCLExpansionResetting
    {
    private:
        double expansion_rate_;
        std::random_device seed_gen_;
        std::default_random_engine engine_;
        std::normal_distribution<> dist_;

    public:
         AMCLExpansionResetting();
        ~ AMCLExpansionResetting();
        virtual double get_entropy(pf_sample_set_t *set);
        virtual void run(pf_sample_set_t *set);
    };

} //namespace amcl

#endif