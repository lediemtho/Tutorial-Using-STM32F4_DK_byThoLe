#ifndef __sensor_h
#define __sensor_h

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "stm32f411e_discovery_gyroscope.h"
#include "lsm303dlhc_driver.h"
#include "lsm303dlhc.h"
#include "math.h"



void GYRO_Calib(void); // read natural value
void XYZ_MAG_Read(int16_t* Mag_Data);   //Read Mag_tu truong
void XYZ_ACC_Read(double* Acc_Data);    //Read acc_gia toc goc
void XYZ_GYRO_Read(double* Gyro_Data);
void XYZ_ANGLE_Read(double* Acc_Data,double* angle); // convert Hex to Degree

# endif
