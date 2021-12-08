
#include "kalman.h"
#include "imudata.h"
#include <stdio.h>


#define KALMAN_ARR_LENGTH 2

enum
{
    KALMAN_X = 0,
    KALMAN_Y,
};


int main()
{
    //IMU_DATA imuData;
    //Imu_Calc_Data imuCalculatedData;

    Kalman_t kalmansArr[KALMAN_ARR_LENGTH];    // Create Kalmans 0. index = KALMAN_X , 1.index = KALMAN_Y
    initKALMANS(kalmansArr,KALMAN_ARR_LENGTH); // Init all created Kalmans (Implemented in Macro Definition and function definition.)
    
    float qAngle;                               // general definition
    setQAngle(kalmansArr[KALMAN_X] , 33.2f); // for Macro Expansion
    getQangle(kalmansArr[KALMAN_X], qAngle); // for Macro Expansion (give qAngle as a parameeter.)

    //setQAngle(&kalmansArr[KALMAN_X], 33.1f);   // for function definition
    //qAngle = getQangle(&kalmansArr[KALMAN_X]); // for function definition

    printf("Float value from kalmanX!: %.2f", qAngle);


    return 0;
}