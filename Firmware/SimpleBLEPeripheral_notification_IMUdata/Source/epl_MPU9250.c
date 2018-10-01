/*
MPU9250 diver
Reference by http://developer.mbed.org/users/kylongmu/code/MPU9250_SPI_Test/
*/

#include <ioCC2541.h>
#include <stdio.h>
#include "OSAL.h"
#include "epl_MPU9250.h"
#include "epl_hal_spi.h"
#include "epl_hal_uart.h"


#define MAG_READ_DELAY 256

float Magnetometer_ASA[3]={0,0,0};
float mRes = 0.15f;
uint16 AK8963_ASA[3] = {0};
#pragma optimize=none
void WAIT_US(uint16 t){   

  uint16 i;
  for ( i = 0; i<t; i++)   
  {
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); 
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); 
  }
}

#pragma optimize=none
void WAIT_MS(uint16 t){                
             
    uint16 i;
    for (i = 0; i<t; i++)
    {
      WAIT_US(250);
      WAIT_US(250);
      WAIT_US(250);
      WAIT_US(250);
    } 
}


#pragma optimize=none
uint8 WriteReg( uint8 WriteAddr, uint8 WriteData )
{	
    uint8 readValue;

    select();
    spiWriteByte(WriteAddr);
    spiWriteByte(WriteData);
    deselect();    
    
    WAIT_US(2);
    spiReadByte(&readValue,0xFF);
	
    return readValue;

}

#pragma optimize=none
uint8 ReadReg( uint8 ReadAddr)
{   
    uint8 readValue;
    select();
    WAIT_US(2);
    spiWriteByte(ReadAddr|READ_FLAG);
    spiReadByte(&readValue,0xFF);
    deselect();

    return readValue;
}

#pragma optimize=none
void ReadRegs( uint8 ReadAddr, uint8 *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;
 
    select();
    spiWriteByte(ReadAddr | READ_FLAG);
    WAIT_US(2);
    for(i=0; i<Bytes; i++)
        spiReadByte(&ReadBuf[i],0xFF);
    deselect();
}

/*=====================================================================================================*
**¨ç¼Æ : MPU9250_Mag_WriteReg
**¥\¯à : AK8963 Write Reg
**¿é¤J : writeAddr, writeData
**¿é¥X : None
**¨Ï¥Î : MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);
**=====================================================================================================*/
/*=====================================================================================================*/

#pragma optimize=none
void MPU9250_Mag_WriteReg( uint8 writeAddr, uint8 writeData )
{
  uint8  status = 0;
  uint32 timeout = MAG_READ_DELAY;

  WriteReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  WAIT_US(1000);
  WriteReg(MPUREG_I2C_SLV4_REG, writeAddr);
  WAIT_US(1000);
  WriteReg(MPUREG_I2C_SLV4_DO, writeData);
  WAIT_US(1000);
  WriteReg(MPUREG_I2C_SLV4_CTRL, MPUREG_I2C_SLVx_EN);
  WAIT_US(1000);

  do {
    status = ReadReg(MPUREG_I2C_MST_STATUS);
    WAIT_US(1000);
  } while(((status & MPUREG_I2C_SLV4_DONE) == 0) && (timeout--));
}


/*=====================================================================================================*/
/*=====================================================================================================*
**¨ç¼Æ : MPU9250_Mag_ReadReg
**¥\¯à : AK8963 Read Reg
**¿é¤J : readAddr
**¿é¥X : readData
**¨Ï¥Î : DeviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
**=====================================================================================================*/
/*=====================================================================================================*/

#pragma optimize=none
uint8 MPU9250_Mag_ReadReg( uint8 readAddr )
{
  uint8 status = 0;
  uint8 readData = 0;
  uint32 timeout = MAG_READ_DELAY;

  WriteReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  WAIT_US(1000);
  WriteReg(MPUREG_I2C_SLV4_REG, readAddr);
  WAIT_US(1000);
  WriteReg(MPUREG_I2C_SLV4_CTRL, MPUREG_I2C_SLVx_EN);
  WAIT_US(1000);

  do {
    status = ReadReg(MPUREG_I2C_MST_STATUS);
    WAIT_US(1000);
  } while(((status & MPUREG_I2C_SLV4_DONE) == 0) && (timeout--));

  readData = ReadReg(MPUREG_I2C_SLV4_DI);

  return readData;
}
/*=====================================================================================================*/
/*=====================================================================================================*
**¨ç¼Æ : MPU9250_Mag_ReadRegs
**¥\¯à : AK8963 Read Regs
**¿é¤J : readAddr, *readData, lens
**¿é¥X : None
**¨Ï¥Î : DeviceID = MPU9250_Mag_ReadRegs(AK8963_WIA);
**=====================================================================================================*/
/*=====================================================================================================*/

#pragma optimize=none
void MPU9250_Mag_ReadRegs( uint8 readAddr, uint8 *readData, uint8 lens )
{
    uint8 status = 0 , i = 0;
    uint32 timeout = MAG_READ_DELAY;

    WriteReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
    WAIT_US(1000);

    for(; i< lens; i++) 
    {
        WriteReg(MPUREG_I2C_SLV4_REG, readAddr + i);
        WAIT_US(1000);
        WriteReg(MPUREG_I2C_SLV4_CTRL, MPUREG_I2C_SLVx_EN);
        WAIT_US(1000);

        do {
            status = ReadReg(MPUREG_I2C_MST_STATUS);
        } while(((status & MPUREG_I2C_SLV4_DONE) == 0) && (timeout--));

        readData[i] = ReadReg(MPUREG_I2C_SLV4_DI);
        WAIT_US(1000);
    }
}
 
/*====================================================================================================*/
/*====================================================================================================*
**¨ç¼Æ : MPU9250_Check
**¥\¯à : Check Device ID
**¿é¤J : None
**¿é¥X : Status
**¨Ï¥Î : Status = MPU9250_Check();
**====================================================================================================*/
/*====================================================================================================*/

#pragma optimize=none
bool MPU9250_Check()
{
    uint8 deviceID = -1;
    char str[40];
    deviceID = ReadReg(MPUREG_WHOAMI);
    if(deviceID != 0x71)
    {   
      sprintf(str,"MPU9250 ID : %x \r\n",deviceID);
      uartWriteString(str);
      return false;
    }
    
    deviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
    if(deviceID != AK8963_Device_ID)
    {
      sprintf(str,"AK8963 e04 ID : %x \r\n",deviceID);
      uartWriteString(str);
      return false;
    }
    
    sprintf(str,"AK8963 ID RRR: %x\r\n",deviceID);
    uartWriteString(str);

    return true;
}
/*====================================================================================================*/
/*====================================================================================================*
**¨ç¼Æ : MPU9250_getData
**¥\¯à : Get IMU Data
**¿é¤J : *dataIMU
**¿é¥X : None
**¨Ï¥Î : MPU9250_getData(dataIMU);
**====================================================================================================*/
/*====================================================================================================*/
void MPU9250_getData( int16 *dataIMU ) //test failed ,wrong value of Magnetometer
{
    uint8 tmpRead[22] = {0};
    
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WAIT_US(10);
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WAIT_US(10);

    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    WAIT_US(20);

    ReadRegs(MPUREG_ACCEL_XOUT_H, tmpRead, 21);
 
    //dataIMU[0] = ((uint16)tmpRead[6] << 8 |  tmpRead[7]);    // Temp
    dataIMU[0] = ((int16)tmpRead[0] << 8 |  tmpRead[1]);  // Acc.X
    dataIMU[1] = ((int16)tmpRead[2] << 8 |  tmpRead[3]);  // Acc.Y
    dataIMU[2] = ((int16)tmpRead[4] << 8 |  tmpRead[5]);  // Acc.Z
    dataIMU[3] = ((int16)tmpRead[8] << 8 |  tmpRead[9]);  // Gyr.X
    dataIMU[4] = ((int16)tmpRead[10] << 8 |  tmpRead[11]);  // Gyr.Y
    dataIMU[5] = ((int16)tmpRead[12] << 8 |  tmpRead[13]);  // Gyr.Z

//    if(!(tmpRead[14] & AK8963_STATUS_DRDY) || (tmpRead[14] & AK8963_STATUS_DOR) || (tmpRead[21] & AK8963_STATUS_HOFL))
//        return;

    dataIMU[6] = ((int16)tmpRead[15] << 8 |  tmpRead[14]);  // Mag.X
    dataIMU[7] = ((int16)tmpRead[17] << 8 |  tmpRead[16]);  // Mag.Y
    dataIMU[8] = ((int16)tmpRead[19] << 8 |  tmpRead[18]);  // Mag.Z

//  dataIMU[7] = ((long)dataIMU[7] * AK8963_ASA[0]) >> 8;
//  dataIMU[8] = ((long)dataIMU[8] * AK8963_ASA[1]) >> 8;
//  dataIMU[9] = ((long)dataIMU[9] * AK8963_ASA[2]) >> 8;
}
/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the DLPF_CFG value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_184HZ
BITS_DLPF_CFG_92HZ
BITS_DLPF_CFG_41HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF

giving the accelerometer DLPF_CFG value; suitable values are:
BITS_ADLPF_1130HZ   
BITS_ADLPF_460HZ      
BITS_ADLPF_184HZ        
BITS_ADLPF_92HZ    
BITS_ADLPF_41HZ
BITS_ADLPF_20HZ    
BITS_ADLPF_5HZ

returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
#define MPU_InitRegNum 13

#define AK8963_InitRegNum   5
 
#pragma optimize=none
bool mpu9250_init(uint8 dlpf_cfg, uint8 a_dlpf_cfg){
        uint8 i = 0;
    //    uint8 sample_div;
    //    uint8 FREQ = 30.0;
        
	P1SEL &= ~0x02;       // 0x1111 1101
        P1DIR |= 0x01;

	spiInit(SPI_MASTER);

      //  sample_div = 1000/FREQ - 1;
	
        if(dlpf_cfg == BITS_DLPF_CFG_8800HZ_NOLPF){
          dlpf_cfg = BITS_DLPF_CFG_184HZ;
          set_gyro_scale(0x00);
        }
    uint8 MPU_Init_Data[MPU_InitRegNum][2] = 
    {
          
      {0x80, MPUREG_PWR_MGMT_1},     // [0]  Reset Device
      {0x04, MPUREG_PWR_MGMT_1},     // [1]  Clock Source
      {0x10, MPUREG_INT_PIN_CFG},    // [2]  Set INT_ANYRD_2CLEAR
      {0x01, MPUREG_INT_ENABLE},     // [3]  Set RAW_RDY_EN
  
      {0x00, MPUREG_PWR_MGMT_2},     // [4]  Enable Acc & Gyro
      {0x00, MPUREG_SMPLRT_DIV},     // [5]  Sample Rate Divider
      {0x18, MPUREG_GYRO_CONFIG},    // [6]  default : +-2000dps
      {0x08, MPUREG_ACCEL_CONFIG},   // [7]  default : +-4G
      {0x07, MPUREG_CONFIG},         // [8]  default : LPS_41Hz
      {0x03, MPUREG_ACCEL_CONFIG_2}, // [9]  default : LPS_41Hz
      {0x30, MPUREG_USER_CTRL},      // [10] Set I2C_MST_EN, I2C_IF_DIS
      {BITS_DLPF_CFG_184HZ, MPUREG_CONFIG},
      {BITS_DLPF_CFG_184HZ, MPUREG_ACCEL_CONFIG_2}
    };
    
    uint8 AK8963_InitData[AK8963_InitRegNum][2] = 
    {
      {0x01, AK8963_CNTL2},           /* [0]  Reset Device                  */
      {0x00, AK8963_CNTL1},           /* [1]  Power-down mode               */
      {0x0F, AK8963_CNTL1},           /* [2]  Fuse ROM access mode          */
                                    /*      Read sensitivity adjustment   */
      {0x00, AK8963_CNTL1},           /* [3]  Power-down mode               */
      {0x06, AK8963_CNTL1},           /* [4]  Continuous measurement mode 2 */
    };

 
    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        WAIT_US(3000);  //I2C must slow down the write speed, otherwise it won't work
    }


    WAIT_MS(3);
    
    bool status = MPU9250_Check();
    if(status != true)
        return false;
    for(i = 0 ; i < 3 ;i++)
    {
      WAIT_MS(3);
      MPU9250_Mag_WriteReg(AK8963_InitData[i][1], AK8963_InitData[i][0]); 
    }
/*read ASA*/
    uint8 tmpRead[3] = {0};

    MPU9250_Mag_ReadRegs(AK8963_ASAX, tmpRead, 3);  // Read sensitivity adjustment values
    WAIT_US(1);
    
    Magnetometer_ASA[0] =  (float)(tmpRead[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    Magnetometer_ASA[1] =  (float)(tmpRead[1] - 128)/256. + 1.;  
    Magnetometer_ASA[2] = (float)(tmpRead[2] - 128)/256. + 1.; 
       
/*read ASA*/
    for(i = 3 ; i < 5 ;i++)
    {
      WAIT_MS(3);
      MPU9250_Mag_WriteReg(AK8963_InitData[i][1], AK8963_InitData[i][0]); 
    }  
       
    
    WriteReg(MPUREG_I2C_MST_CTRL, 0x5D);
    WAIT_US(2000);
    WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    WAIT_US(2000);
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ST1);
    WAIT_US(2000);
    WriteReg(MPUREG_I2C_SLV0_CTRL, MPUREG_I2C_SLVx_EN | 8);
    WAIT_US(2000);
    WriteReg(MPUREG_I2C_SLV4_CTRL, 0x09);
    WAIT_US(2000);
    WriteReg(MPUREG_I2C_MST_DELAY_CTRL, 0x81);
    WAIT_MS(300);


    char str[40];
    int deviceID = MPU9250_Mag_ReadReg(AK8963_WIA);    
    sprintf(str,"AK8963 ID : %x \r\n",deviceID);
    uartWriteString(str);
   //AK8963_calib_Magnetometer();
    return true;
}
void get_Magnetometer_ASA(float *magCalibration)
{
  magCalibration[0]=Magnetometer_ASA[0];
  magCalibration[1]=Magnetometer_ASA[1];
  magCalibration[2]=Magnetometer_ASA[2];
  
}

void magcalMPU9250(float * dest1, float * dest2) 
{
  uint16 ii = 0, sample_count = 0;
  int32 mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16 mag_max[3] = {-32760, -32760, -32760}, mag_min[3] = {32760, 32760, 32760}, mag_temp[3] = {0, 0, 0};
  char bufASA[60] = {0};
  
  uartWriteString("Mag Calibration: Wave device in a figure eight until done!");
  WAIT_MS(4000);
  
  // shoot for ~fifteen seconds of mag data
  sample_count = 1000;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    read_Mag(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    WAIT_MS(11);  // at 100 Hz ODR, new mag data is available every 10 ms
  }
  
  //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
  
  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
  dest1[0] = (float) mag_bias[0]*mRes*Magnetometer_ASA[0];  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1]*mRes*Magnetometer_ASA[1];   
  dest1[2] = (float) mag_bias[2]*mRes*Magnetometer_ASA[2];  
  
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  
  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
  
  uartWriteString("Mag Calibration done!\r\n");
  
  sprintf (bufASA, "mag MAX:  x: %hd\t,  y: %hd\t,  z: %hd\r\n", mag_max[0], mag_max[1], mag_max[2]);
  uartWriteString(bufASA);
  osal_memset(bufASA, 0, 60);
  sprintf (bufASA, "mag MIN:  x: %hd\t,  y: %hd\t,  z: %hd\r\n", mag_min[0], mag_min[1], mag_min[2]);
  uartWriteString(bufASA);
 
  
}

/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
-----------------------------------------------------------------------------------------------*/
void set_acc_scale(int scale){    
    WriteReg(MPUREG_ACCEL_CONFIG, scale);   
}

uint8 get_acc_scale(){
	unsigned int scale;
	
	scale=ReadReg(MPUREG_ACCEL_CONFIG);
	
	switch(scale & 0x18){
		case BITS_FS_2G:  return 2;
		case BITS_FS_4G:  return 4;
		case BITS_FS_8G:  return 8;
		case BITS_FS_16G: return 16;
	}
	
	return 0;
}

float get_acc_divider(){
	unsigned int scale;
	float acc_divider;
	
	scale=ReadReg(MPUREG_ACCEL_CONFIG);
	
	    switch (scale & 0x18){
			case BITS_FS_2G:
				acc_divider=16384;
			break;
			case BITS_FS_4G:
				acc_divider=8192;
			break;
			case BITS_FS_8G:
				acc_divider=4096;
			break;
			case BITS_FS_16G:
				acc_divider=2048;
			break;   
		}
		
	return acc_divider;
}

 
/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
-----------------------------------------------------------------------------------------------*/
void set_gyro_scale(int scale){
    WriteReg(MPUREG_GYRO_CONFIG, scale);
}

uint16 get_gyro_scale(){
	unsigned int scale;
	
	scale=ReadReg(MPUREG_GYRO_CONFIG);

	switch (scale){
        case BITS_FS_250DPS : return 250;
        case BITS_FS_500DPS : return 500;
        case BITS_FS_1000DPS: return 1000;
        case BITS_FS_2000DPS: return 2000;
    }
	
	return 250;
}

float get_gyro_divider(){
	unsigned int scale;
	float gyro_divider;
	
	scale=ReadReg(MPUREG_GYRO_CONFIG);

	switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;   
    }
	
	return gyro_divider;
}
 
 
/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu9250 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
uint8 whoami(){
    uint8 response;
    response=ReadReg(MPUREG_WHOAMI);
    return response;
}
 
 
/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER(Need Verify)
usage: call this function to read accelerometer data.
return the accelerometer data in uint16 format

-----------------------------------------------------------------------------------------------*/

int16 read_accX()
{
	uint8 xout_h, xout_l;
	
	xout_h = ReadReg(MPUREG_ACCEL_XOUT_H);
	xout_l = ReadReg(MPUREG_ACCEL_XOUT_L);
	
	return ((int16)xout_h<<8)|xout_l;
}

int16 read_accY()
{
	uint8 yout_h, yout_l;
	
	yout_h = ReadReg(MPUREG_ACCEL_YOUT_H);
	yout_l = ReadReg(MPUREG_ACCEL_YOUT_L);
	
	return ((int16)yout_h<<8)|yout_l;
}

int16 read_accZ()
{
	uint8 zout_h, zout_l;
	
	zout_h = ReadReg(MPUREG_ACCEL_ZOUT_H);
	zout_l = ReadReg(MPUREG_ACCEL_ZOUT_L);
	
	return ((int16)zout_h<<8)|zout_l;
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
return the gyroscope data in uint16 format

-----------------------------------------------------------------------------------------------*/
uint16 read_rotX()
{
	uint8 xout_h, xout_l;
	
	xout_h = ReadReg(MPUREG_GYRO_XOUT_H);
	xout_l = ReadReg(MPUREG_GYRO_XOUT_L);
	
	return ((int16)xout_h<<8)|xout_l;
}

uint16 read_rotY()
{
	uint8 yout_h, yout_l;
	
	yout_h = ReadReg(MPUREG_GYRO_YOUT_H);
	yout_l = ReadReg(MPUREG_GYRO_YOUT_L);
	
	return ((int16)yout_h<<8)|yout_l;
}

uint16 read_rotZ()
{
	uint8 zout_h, zout_l;
	
	zout_h = ReadReg(MPUREG_GYRO_ZOUT_H);
	zout_l = ReadReg(MPUREG_GYRO_ZOUT_L);
	
	return ((int16)zout_h<<8)|zout_l;
}
 
/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in float
-----------------------------------------------------------------------------------------------*/
void read_temp(float* Temperature){
    uint8 tem_h;
	uint8 tem_l;
    int16 bit_data;
    float data;
    
	tem_h = ReadReg(MPUREG_TEMP_OUT_H);
	tem_l = ReadReg(MPUREG_TEMP_OUT_L);
	
    bit_data=((int16)tem_h<<8)|tem_l;
    data=(float)bit_data;
    *Temperature=(data/340)+36.53;
    deselect();
}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE in BYTE
usage: call this function to read temperature data. 
returns the value in uint16
-----------------------------------------------------------------------------------------------*/

uint16 read_temp_byte(){
	uint8 tem_h, tem_l;
	
	tem_h = ReadReg(MPUREG_TEMP_OUT_H);
	tem_l = ReadReg(MPUREG_TEMP_OUT_L);
	
	return ((uint16)tem_h<<8)|tem_l;
}
 
/*Calibration*/

void calibrateMPU9250(float * gyroBias, float * accBias)
{  
  uint16 i = 0;
  int16 data[9];
  int32 gyro_zero[3]={0}, acc_zero[3]={0};
    
  float gyrosensitivity = get_gyro_divider();
  float accelsensitivity = get_acc_divider();
  
  int count = 3000;  
  for(i = 0;i < count ; i++)
  {
    MPU9250_getData(data);
    acc_zero[0] += data[0];
    acc_zero[1] += data[1];
    acc_zero[2] += data[2];
    
    gyro_zero[0] += data[3];
    gyro_zero[1] += data[4];
    gyro_zero[2] += data[5];
    
    WAIT_US(50);
  }

  for(i=0;i<3;i++)
  {
     acc_zero[i] /= count;
     gyro_zero[i] /= count;
  }
  
  if(acc_zero[2] > 0) acc_zero[2] -= (int32)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  else acc_zero[2] += (int32)accelsensitivity;
  
  for(i=0;i<3;i++){
     accBias[i]  = (float)acc_zero[i]/(float)accelsensitivity;
     gyroBias[i] = (float)gyro_zero[i]/(float)gyrosensitivity;
  }
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION(Need Verify)
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void calib_acc(int* calib_data)
{
    uint8 response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=ReadReg(MPUREG_ACCEL_CONFIG);
    set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);
 
    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    calib_data[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    calib_data[2]=((response[2]&11100000)>>3)|((response[3]&00000011));
 
    set_acc_scale(temp_scale);
}

uint8 AK8963_whoami(){
    uint8 response;

    WriteReg(MPUREG_I2C_SLV0_ADDR,0x8C); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
 
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    WAIT_US(50);
    response=ReadReg(MPUREG_EXT_SENS_DATA_00);    //Read I2C
    return response;
}
#pragma optimize=none
void read_Mag(int16* Magnetometer){
    uint8 response[7];
  //  int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WAIT_US(1000);
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WAIT_US(1000);

    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
 
    WAIT_US(1000);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    
    Magnetometer[0] = (int16)response[1] << 8 | response[0]; 
    Magnetometer[1] = (int16)response[3] << 8 | response[2]; 
    Magnetometer[2] = (int16)response[5] << 8 | response[4]; 
    
    return;
}

#pragma optimize=none
uint16 read_magX(){
    uint8 response[7];
    uint16 value;
    //int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
 
    WAIT_US(50);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
   
    value = (int16)response[1]<<8 | response[0];   
    
    return value;
}

uint16 read_magY(){
    uint8 response[7];
    uint16 value;
  //  int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 2 bytes from the magnetometer
 
    WAIT_US(50);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    value = (int16)response[3]<<8 | response[2];
    
    return value;
}

uint16 read_magZ(){
    uint8 response[7];
    uint16 value;
 //   int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 2 bytes from the magnetometer
 
    WAIT_US(50);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    value = (int16)response[5]<<8 | response[4];
    
    return value;
}



/*Enter into low power mode*/
void MPU9250_low_power(){
     
    uint8 i; 
  
    uint8 MPU_Init_Data[8][2] = {
        {0x10, MPUREG_PWR_MGMT_1},    //      
        {0x03, MPUREG_PWR_MGMT_1}, 
        {0x09, MPUREG_ACCEL_CONFIG_2},
        {0x40, MPUREG_INT_ENABLE}, //  Enables multi-master  IIC 400KHz
        {0xC0, MPUREG_MOT_DETECT_CTRL},       // Enable AUX and make SPI only    
        {0x77, MPUREG_MOT_THR},  //Set the I2C slave addres of AK8963 and set for read.
        {0x0B, MPUREG_LP_ACCEL_ODR}, //I2C slave 0 register address from where to begin data transfer
        {0x30, MPUREG_PWR_MGMT_1},  //Enable I2C and set bytes

    };
 
    for(i=0; i<8; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        WAIT_US(2);  //I2C must slow down the write speed, otherwise it won't work
    }

}




/*-----------------------------------------------------------------------------------------------
                                SPI SELECT AND DESELECT
usage: enable and disable mpu9250 communication bus
-----------------------------------------------------------------------------------------------*/
void select() {
    //Set CS low to start transmission (interrupts conversion)
    CS = CS_ENABLED;
}
void deselect() {
    //Set CS high to stop transmission (restarts conversion)
    CS = CS_DISABLED;
}

void readRegs()
{
  int i;
  char str[100];
  
  for(i = 0x19 ; i < 0x3B ; i++)
  {
    uint8  retVal = ReadReg(i);
    sprintf(str,"register %x value: %x \r\n",i,retVal);
    uartWriteString(str);
    WAIT_US(20);
  }
}

            