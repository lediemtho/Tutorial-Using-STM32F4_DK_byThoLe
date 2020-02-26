#include "sensor.h"

int16_t gyro_calib[3]={0};

/*Calibration gyro*/
void GYRO_Calib()
{

	int16_t gyro_sum[3]={0}, gyro_read[3]={0};
	//calibration gyroscope
	for (uint16_t i=0; i<50;i++)
		{
			L3GD20_ReadXYZAngRate(gyro_read);
			for ( uint8_t index=0; index<3; index++)
				{
					gyro_sum[index] += gyro_read[index];
				}
			HAL_Delay(100);				
		}
	for ( uint8_t i=0; i<3; i++)
		{
			gyro_calib[i] = gyro_sum[i]/50;
		}
}

/*L3GD20: Calculating raw value*/
	void XYZ_GYRO_Read(double* Gyro_Data)		//Unit: dps (degree per second)
	{
		int16_t gyro_read[3] = {0}; 
		L3GD20_ReadXYZAngRate(gyro_read);		

		for(uint8_t i=0; i<3; i++)
			{
				Gyro_Data[i]=(double)((gyro_read[i]-gyro_calib[i])*0.0152590219 );			//Resolution = 1000/(2^16-1)
			} 
	}
	
	//LSM303DLHC: Accelerometer
	void XYZ_ACC_Read(double* Acc_Data) 
	{
	int16_t acc_read[3]={0};
	LSM303DLHC_AccReadXYZ(acc_read);
	for(uint8_t i=0; i<3; i++)
			{
			Acc_Data[i]= (double)((acc_read[i])*0.00006103608);  
				/* 4/(2^16 - 1) , do phan giai
				full scale +-2 --> tin hieu luong cuc 4 8 */
			}	
	}
	
	// calculate Angle
	
	void XYZ_ANGLE_Read(double* Acc_Data,double* angle)
{
	//Roll
	angle[0] = atan2(Acc_Data[1],Acc_Data[2])*180/3.1416;	
	//Pitch
	angle[1] = atan2(Acc_Data[0],Acc_Data[2])*180/3.1416;	
	//Yaw
	angle[2] = atan2(Acc_Data[0],Acc_Data[1])*180/3.1416;	
	
	}

	