#pragma once

#include <inttypes.h>


/* If you dont want to use Macro expansions Please comment it that line.
*/
#define USE_MACRO_EXPANSION 1

#ifdef USE_MACRO_EXPANSION
#define setAngle( kalman, angle_ )                   ( kalman.angle     = angle_ ) // Used to set angle, this should be set as the starting angle
#define setQAngle( kalman, Q_angle_ )                ( kalman.Q_angle   = Q_angle_ )


/*
 * setQbias(Kalman_t kalman , float Q_bias)
 * Default value (0.003f) is in Kalman.h
 *      ->Raise this to follow input more closely,
 *      ->lower this to smooth result of kalman filter.
 */
#define setQbias( kalman, Q_bias_ )                  ( kalman.Q_bias    = Q_bias_ )
#define setRmeasure( kalman, R_measure_ )            ( kalman.R_measure = R_measure_ )
#define getQangle( kalman, returnQangle )            ( returnQangle     = kalman.Q_angle )
#define getQbias( kalman, returnQbias )              ( returnQbias      = kalman.Q_bias )
#define getRmeasure( kalman, returnRmeasure )        ( returnRmeasure)  = kalman.R_measure )
#define getRate( kalman, returnRate )                ( returnRate       = kalman.rate ) // Return the unbiased rate
#define getAngle( kalman, newAngle, newRate, dt, returnAngle )  do {\
                                                                    kalman.rate = newRate - kalman.bias;\
                                                                    kalman.angle += dt * kalman.rate;\
                                                                    kalman.P[0][0] += dt * (dt*kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + kalman.Q_angle);\
                                                                    kalman.P[0][1] -= dt * kalman.P[1][1];\
                                                                    kalman.P[1][0] -= dt * kalman.P[1][1];\
                                                                    kalman.P[1][1] += kalman.Q_bias * dt;\
                                                                    float S = kalman.P[0][0] + kalman.R_measure;\
                                                                    float K[2];\
                                                                    K[0] = kalman.P[0][0] / S;\
                                                                    K[1] = kalman.P[1][0] / S;\
                                                                    float y = newAngle - kalman.angle;\
                                                                    kalman.angle += K[0] * y;\
                                                                    kalman.bias  += K[1] * y;\
                                                                    float P00_temp = kalman.P[0][0];\
                                                                    float P01_temp = kalman.P[0][1];\
                                                                    kalman.P[0][0] -= K[0] * P00_temp;\
                                                                    kalman.P[0][1] -= K[0] * P01_temp;\
                                                                    kalman.P[1][0] -= K[1] * P00_temp;\
                                                                    kalman.P[1][1] -= K[1] * P01_temp;\
                                                                    returnAngle = angle;}while(0);


 /*      Q_angle
         Q_bias
         R_measure
     Those variables can tuned by the user.

     Resetted the angle.
     Resetted the bias.
   Since we assume that the bias is 0 and we know the starting angle
   (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

 */
#define initKalman( kalman )                do{\
                                                kalman.Q_angle = 0.001f;\
                                                kalman.Q_bias = 0.003f;\
                                                kalman.R_measure = 0.03f;\
                                                kalman.angle = 0.0f;\
                                                kalman.bias = 0.0f;\
                                                kalman.P[0][0] = 0.0f;\
                                                kalman.P[0][1] = 0.0f;\
                                                kalman.P[1][0] = 0.0f;\
                                                kalman.P[1][1] = 0.0f;}while(0); // Reset the kalman
#endif


typedef struct
{
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
}Kalman_t;

extern void initKALMANS(Kalman_t* const kalmanArray, uint8_t length);

#ifndef USE_MACRO_EXPANSION

extern void initKalman( Kalman_t* const kalman );
extern void setAngle( Kalman_t* const kalman, const float newAngle );
extern void setQAngle( Kalman_t* const kalman, const float newQangle );
extern void setQbias( Kalman_t* const kalman, const float newQbias );
extern void setRmeasure( Kalman_t* const kalman, const float newRmeasure );

extern float getQangle( const Kalman_t* const kalman );
extern float getQbias( const Kalman_t* const kalman );
extern float getRmeasure( const Kalman_t* const kalman );
extern float getRate(  const Kalman_t* const kalman );
extern float getAngle( Kalman_t* const kalman, const float newAngle, const float newRate, const float dt );


#endif

