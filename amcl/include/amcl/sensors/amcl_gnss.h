//ER+GR
#ifndef AMCL_GNSS_H
#define AMCL_GNSS_H

#include "amcl_sensor.h"

namespace amcl{
    class GnssSensorData : public AMCLSensorData{
    public:
        double gnss_x;
        double gnss_y;
        int measurement_solve;
    };

    class AMCLGnssSensor : public GnssSensorData{
    private:
    public:
        AMCLGnssSensor();
        virtual void initGnssData(GnssSensorData *gnss_data_t);
        virtual void calKld(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t);
        virtual void er(pf_t *pf, amcl_state *state_t);
        virtual void ergr(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t);
    };
}
#endif
//ER+GR