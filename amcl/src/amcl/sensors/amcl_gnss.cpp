//ER+GR
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <ctime>
#include <cstdlib>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "amcl/sensors/amcl_gnss.h"

using namespace amcl;

AMCLGnssSensor :: AMCLGnssSensor(){
    std::srand( std::time(NULL) );
}

void AMCLGnssSensor ::initGnssData(GnssSensorData *gnss_data_t){
	gnss_data_t->gnss_x = 0;
	gnss_data_t->gnss_y = 0;
}

void AMCLGnssSensor :: calKld(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t){
	int d = 2; //次元数　GNSSの計測値に角度がないためx, yの二次元でKL情報量を計算する
	Eigen::Vector2d amcl_position_t;
	Eigen::Matrix2d amcl_sigma;

	Eigen::Vector2d gnss_position_t;
	Eigen::Matrix2d	gnss_sigma;


	amcl_position_t << state_t->max_weight_pose[0], state_t->max_weight_pose[1];
	amcl_sigma << state_t->particle_sigma[0], 0,
				  0, state_t->particle_sigma[1];

	gnss_position_t << gnss_data_t->gnss_x, gnss_data_t->gnss_y;
	gnss_sigma << pf->reset_gnss_sigma[0], 0,
				  0, pf->reset_gnss_sigma[1];

	double kld = ( log(gnss_sigma.determinant() / amcl_sigma.determinant() )
					+ (gnss_sigma.inverse() * amcl_sigma).trace()
						+ (amcl_position_t - gnss_position_t).transpose() * gnss_sigma.inverse() * (amcl_position_t - gnss_position_t)
							- d ) / 2;
    //double kld = log(10) ;
	state_t->kld_t = kld;
    // std::cout << state_t->particle_sigma[0] << "\t" << state_t->particle_sigma[0] << std::endl;
	// std::cout << pf->reset_gnss_sigma[0] << "\t" << pf->reset_gnss_sigma[1] << std::endl;
	// std::cout << state_t->max_weight_pose[0] << "\t" << state_t->max_weight_pose[1] << "\t"
	// 	<< gnss_data_t->gnss_x << "\t" << gnss_data_t->gnss_y << "\t" << kld << std::endl;
}

void AMCLGnssSensor :: er(pf_t *pf, amcl_state *state_t){
    int reset_count = 0;
	pf_sample_set_t *set;
	pf_sample_t *sample;

	set = pf->sets + pf->current_set;

	int reset_limit = ( (int)state_t->particle_sigma[0] + (int)state_t->particle_sigma[1]) / 2;

    if(reset_count >= reset_limit){
		for(int i=0; i<set->sample_count; i++){
            sample = set->samples + i;
			sample->pose.v[0] += (drand48() * 4 * state_t->particle_sigma[0]) - (2 * state_t->particle_sigma[0]);
			sample->pose.v[1] += (drand48() * 4 * state_t->particle_sigma[1]) - (2 * state_t->particle_sigma[1]);
			sample->pose.v[2] += (drand48() * 2 * state_t->particle_sigma[2]) - (1 * state_t->particle_sigma[2]);

			sample->weight = 1.0 / set->sample_count;
		}
    reset_count=0;
	}
    reset_count++;

}

void AMCLGnssSensor :: ergr(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t){
    int reset_count = 0;
	pf_sample_set_t *set;
	pf_sample_t *sample;

	set = pf->sets + pf->current_set;
	int reset_particle_num = state_t->particle_num * state_t->beta / (1 + state_t->beta);	//GNSS計測展に置くパーティクルの数
	double particle_sigma_sum = (state_t->particle_sigma[0] + state_t->particle_sigma[1]) / 2;


	if(particle_sigma_sum > pf->sigma_th && state_t->kld_t > pf->kld_th){
		// GRを行う
		// 論文では角度情報は全て棄却していたが、つくばで使用する場合は逆に誘拐状態になる可能性があるため
		// 棄却を行わない
		for(int i=0; i<reset_particle_num; i++){
            sample = set->samples + i;
			sample->pose.v[0] = gnss_data_t->gnss_x + (drand48()*2*pf->reset_gnss_sigma[0] - pf->reset_gnss_sigma[0]);
			sample->pose.v[1] = gnss_data_t->gnss_y + (drand48()*2*pf->reset_gnss_sigma[1] - pf->reset_gnss_sigma[1]);
		}
	}else{
		int reset_limit = ( (int)state_t->particle_sigma[0] + (int)state_t->particle_sigma[1]) / 2;
		if(reset_count >= reset_limit){
			for(int i=0; i<set->sample_count; i++){
                sample = set->samples + i;
				sample->pose.v[0] += (drand48() * 4 * state_t->particle_sigma[0]) - (2 * state_t->particle_sigma[0]);
				sample->pose.v[1] += (drand48() * 4 * state_t->particle_sigma[1]) - (2 * state_t->particle_sigma[1]);
				sample->pose.v[2] += (drand48() * 2 * state_t->particle_sigma[2]) - (1 * state_t->particle_sigma[2]);

				sample->weight = 1.0 / set->sample_count;
			}
        reset_count = 0;
		}
    reset_count++;
	}
}
//ER+GR