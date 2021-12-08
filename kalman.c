#include "kalman.h"



void initKALMANS(Kalman_t* const kalmanArray, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        #ifdef USE_MACRO_EXPANSION
            initKalman(kalmanArray[i]);
        #else
            initKalman(&kalmanArray[i]);
        #endif  
    }
};


#ifndef USE_MACRO_EXPANSION
float getQangle(const Kalman_t* const kalman)
{
    return kalman->Q_angle;
}

float getQbias(const Kalman_t* const kalman)
{
    return kalman->Q_bias;
}

float getRmeasure(const Kalman_t* const kalman)
{
    return kalman->R_measure;
}

float getRate(const Kalman_t* const kalman)
{
    return kalman->rate;
}

void initKalman( Kalman_t* const kalman )
{
    kalman->Q_angle   = 0.001f; 
    kalman->Q_bias    = 0.003f;
    kalman->R_measure = 0.03f;
    kalman->angle     = 0.0f;
    kalman->bias      = 0.0f;
    kalman->P[0][0]   = 0.0f;
    kalman->P[0][1]   = 0.0f;
    kalman->P[1][0]   = 0.0f;
    kalman->P[1][1]   = 0.0f;
}
void setAngle( Kalman_t* const kalman, const float newAngle )
{
    kalman->angle = newAngle;
}
void setQAngle( Kalman_t* const kalman, const float newQangle )
{
    kalman->Q_angle = newQangle;
}
void setQbias( Kalman_t* const kalman, const float newQbias )
{
    kalman->Q_bias = newQbias;
}
void setRmeasure( Kalman_t* const kalman, const float newRmeasure )
{
    kalman->R_measure = newRmeasure;
}


float getAngle( Kalman_t* const kalman, const float newAngle, const float newRate, const float dt )
{
    kalman->rate = newRate - kalman->bias; 
        kalman->angle += dt * kalman->rate; 
        kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle); 
        kalman->P[0][1] -= dt * kalman->P[1][1]; 
        kalman->P[1][0] -= dt * kalman->P[1][1]; 
        kalman->P[1][1] += kalman->Q_bias * dt; 
        float S = kalman->P[0][0] + kalman->R_measure; 
        float K[2]; 
        K[0] = kalman->P[0][0] / S; 
        K[1] = kalman->P[1][0] / S; 
        float y = newAngle - kalman->angle; 
        kalman->angle += K[0] * y; 
        kalman->bias += K[1] * y; 
        float P00_temp = kalman->P[0][0]; 
        float P01_temp = kalman->P[0][1]; 
        kalman->P[0][0] -= K[0] * P00_temp; 
        kalman->P[0][1] -= K[0] * P01_temp; 
        kalman->P[1][0] -= K[1] * P00_temp; 
        kalman->P[1][1] -= K[1] * P01_temp; 
        return kalman->angle;
}
#endif
