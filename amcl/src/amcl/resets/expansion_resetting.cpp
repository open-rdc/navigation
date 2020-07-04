#include "amcl/resets/expansion_resetting.h"

using namespace amcl;

AMCLExpansionResetting::AMCLExpansionResetting():engine_(seed_gen_())
{
    expansion_rate_ = 0.2;

    std::normal_distribution<>::param_type param(0.0, expansion_rate_);
    dist_.param(param);

}

void AMCLExpansionResetting::run(pf_sample_set_t *set){
    pf_sample_t *sample;

    for(int i=0; i<set->sample_count; i++){
        sample = set->samples + i;
        sample->pose.v[0] += dist_(engine_);
        sample->pose.v[1] += dist_(engine_);
        sample->pose.v[2] += dist_(engine_);

        sample->weight = 1.0 / set->sample_count;
    }

}

AMCLExpansionResetting::~AMCLExpansionResetting(){

}
