#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "amcl/pf/pf.h"
#include "amcl/resets/gnss_resetting.h"

using namespace amcl;

AMCLGnssResetting::AMCLGnssResetting():engine_(seed_gen_())
{
    reset_rate_ = 0.05;

    std::normal_distribution<>::param_type param_x(0.0, reset_rate_);
    std::normal_distribution<>::param_type param_y(0.0, reset_rate_);
    std::uniform_real_distribution<>::param_type param_theta(-M_PI, M_PI);
    dist_x.param(param_x);
    dist_y.param(param_y);
    dist_theta.param(param_theta);
}

AMCLGnssResetting::~AMCLGnssResetting()
{
}

double
AMCLGnssResetting::calc_kl_divergence(pf_sample_set_t *set, const gnss_t gnss){
    Eigen::Vector2d pf_position;
	Eigen::Matrix2d pf_cov;

	Eigen::Vector2d gnss_position;
	Eigen::Matrix2d	gnss_cov;    
    
    pf_position << set->mean.v[0],
                   set->mean.v[1];

    pf_cov << set->cov.m[0][0],                0,                 
              0,                set->cov.m[1][1];

    gnss_position << gnss.pose.v[0],
                     gnss.pose.v[1];

    gnss_cov << gnss.cov.m[0][0],                0,                 
                0,                gnss.cov.m[1][1];

    int d = pf_position.size();

    kl_divergence_ =  ( std::log(gnss_cov.determinant()/pf_cov.determinant()) + (gnss_cov.inverse() * pf_cov).trace() + (gnss_position - pf_position).transpose() * gnss_cov.inverse() * (gnss_position - pf_position) -d )/2;

    return kl_divergence_;
}

double
AMCLGnssResetting::calc_nd_pdf(gnss_t gnss0, gnss_t gnss1){
    return 0;
}

void
AMCLGnssResetting::sampling(pf_sample_set_t *set, const gnss_t gnss){
	pf_sample_t *sample;

    std::normal_distribution<>::param_type param_x(0.0, std::sqrt(gnss.cov.m[0][0]));
    std::normal_distribution<>::param_type param_y(0.0, std::sqrt(gnss.cov.m[1][1]));

    for(int i=0; i<set->sample_count; i++){
            sample = set->samples + i;
			sample->pose.v[0] = gnss.pose.v[0] + dist_x(engine_, param_x);
			sample->pose.v[1] = gnss.pose.v[1] + dist_y(engine_, param_y);
            sample->pose.v[2] = gnss.pose.v[1] + dist_theta(engine_);
            sample->weight = 1.0 / set->sample_count;
	}
}

