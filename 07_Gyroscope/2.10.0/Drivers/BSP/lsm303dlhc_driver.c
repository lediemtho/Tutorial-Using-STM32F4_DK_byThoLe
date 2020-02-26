/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : LSM303DLHC.c
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LSM303DLHC driver file
*                      
* HISTORY:
* Date               |	Modification                    |	Author
* 02/08/2011         |	Initial Revision                |	Fabio Tota

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lsm303dlhc_driver.h"
#include "lsm303dlhc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*			: I2C or SPI reading functions					
* Input			: Register Address
* Output		: Data REad
* Return		: None
*******************************************************************************/
u8_t ReadReg(u8_t deviceAddr, u8_t Reg) {
  
  //To be completed with either I2c or SPI reading function
  //*Data = SPI_Mems_Read_Reg( Reg );
	return COMPASSACCELERO_IO_Read(deviceAddr, Reg);
}


/*******************************************************************************
* Function Name		: WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t WriteReg(u8_t deviceAddress, u8_t WriteAddr, u8_t Data) {
    
  //To be completed with either I2c or SPI writing function
  //SPI_Mems_Write_Reg(Reg, Data);
  COMPASSACCELERO_IO_Write(deviceAddress, WriteAddr, Data);
  return 1;
}


/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Function Name  : SetODR_M
* Description    : Sets LSM303DLHC Output Data Rate Magnetometer
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetODR_M(ODR_M_t ov){
  u8_t value;

  value &= 0x80; //bit<6,5,1,0> must be =0 for correct working
  value |= ov<<ODR_M;

  if( !WriteReg(MAG_I2C_ADDRESS, CRA_REG_M, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetTemperature
* Description    : Sets LSM303DLHC Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetTemperature(State_t state){
  u8_t value;

  value &= 0x7F;
  value |= state<<TEMP_EN;

  if( !WriteReg(MAG_I2C_ADDRESS, CRA_REG_M, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetGainMag
* Description    : Sets LSM303DLHC Magnetometer Gain
* Input          : GAIN_1100_M or GAIN_855_M or GAIN_670_M or GAIN_450_M....
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetGainMag(GAIN_M_t Gain){
  u8_t value;
  value &= 0x00; //bit<4-0> must be =0 for correct working
  value |= Gain<<GN_CFG;

  if( !WriteReg(MAG_I2C_ADDRESS, CRB_REG_M, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetModeMag
* Description    : Sets LSM303DLHC Magnetometer Modality
* Input          : Modality (CONTINUOUS_MODE, or SINGLE_MODE, or SLEEP_MODE)	
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetModeMag(Mode_M_t Mode){
  u8_t value;

  
  value &= 0x00; //bit<7-3> must be =0 for correct working
  value |= Mode<<MODE_SEL_M;

  if( !WriteReg(MAG_I2C_ADDRESS, MR_REG_M, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}




/*******************************************************************************
* Function Name  : GetMagAxesRaw
* Description    : Read the Magnetometer Values Output Registers
* Input          : buffer to empity by MagAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void LSM303DLHC_MagReadXYZ(i16_t* buff)
{
  u8_t valueL;
  u8_t valueH;  
  valueL = ReadReg(MAG_I2C_ADDRESS, OUT_X_L_M) ;  
  valueH = ReadReg(MAG_I2C_ADDRESS, OUT_X_H_M);  
  buff[0] = (i16_t)( (valueH << 8) | valueL )/16;
	valueL = ReadReg(MAG_I2C_ADDRESS, OUT_Y_L_M);
	valueH = ReadReg(MAG_I2C_ADDRESS, OUT_Y_H_M);
  buff[1] = (i16_t)( (valueH << 8) | valueL )/16;  
  valueL = ReadReg(MAG_I2C_ADDRESS, OUT_Z_L_M);
  valueH = ReadReg(MAG_I2C_ADDRESS, OUT_Z_H_M);
  buff[2] = (i16_t)( (valueH << 8) | valueL )/16;
}
/*******************************************************************************
* Function Name  : SetMode
* Description    : Sets LSM303DLHC Operating Mode Accelrometer
* Input          : Modality (NORMAL, LOW_POWER, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t SetMode(Mode_t md) {
  u8_t value;
  u8_t value2;
  static   u8_t ODR_old_value;
 
  if((value & 0xF0)==0) value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN  
    
  switch(md) {
  
  case POWER_DOWN:
    ODR_old_value = value;
    value &= 0x0F;
    break;
          
  case NORMAL:
    value &= 0xF7;
    value |= (MEMS_RESET<<LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_SET<<HR);   //set HighResolution_BIT
    break;
          
  case LOW_POWER:		
    value &= 0xF7;
    value |=  (MEMS_SET<<LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_RESET<<HR); //reset HighResolution_BIT
    break;
          
  default:
    return MEMS_ERROR;
  }
  
  if( !WriteReg(ACC_I2C_ADDRESS, CTRL_REG1_A, value) )
    return MEMS_ERROR;
  
  if( !WriteReg(ACC_I2C_ADDRESS, CTRL_REG4_A, value2) )
    return MEMS_ERROR;  
   
  return MEMS_SUCCESS;
}

