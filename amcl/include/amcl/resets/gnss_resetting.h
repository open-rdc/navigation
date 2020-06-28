#ifndef GNSS_RESETTING_H
#define GNSS_RESETTING_H

#include <random>
#include "amcl/pf/pf.h"

namespace amcl{

typedef struct
{
  double v[2];
} gnss_vector_t;

typedef struct _gnss_matrix_t
{
  double m[2][2];
} gnss_matrix_t;

typedef struct _gnss_t
{
    gnss_vector_t pose;
    gnss_matrix_t cov;
    double weight;
}gnss_t;

class AMCLGnssResetting
{
private:
double reset_rate_;
double kl_divergence_;
std::random_device seed_gen_;
std::default_random_engine engine_;
std::normal_distribution<> dist_;
    
public:
    AMCLGnssResetting();
    ~AMCLGnssResetting();
    virtual double calc_kl_divergence(pf_t *pf, const gnss_t gnss);
    virtual void run(pf_t *pf, const gnss_t gnss);
};

}//namespace amcl

#endif