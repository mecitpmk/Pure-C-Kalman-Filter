#pragma once

#include <inttypes.h>

typedef struct
{
	float accX		 ;
	float accY		 ;
	float accZ		 ;

	float gyroX		 ;
	float gyroY		 ;
	float gyroZ		 ;

	uint16_t tempRaw ;

}IMU_DATA;



typedef struct
{
	float gyroXangle ;
	float compXangle ;
	float kalXangle  ;

	float gyroYangle ;
	float compYangle ;
	float kalYangle  ;

}Imu_Calc_Data;


