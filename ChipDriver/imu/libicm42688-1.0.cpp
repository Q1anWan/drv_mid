/*
 * @Description: A instantiation of IMU-ICM42688
 * @Author: qianwan
 * @Date: 2023-10-30 18:50:25
 * @LastEditTime: 2023-11-10 10:59:09
 * @LastEditors: qianwan
 */

 /******************************************************************************
  * @attention
  * BSD 3-Clause License
  * Copyright (c) 2023, Qianwan.Jin
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************/
 /*Version 1.0*/
 /*Stepper 0.0*/
#include "libicm42688-1.0.hpp"
using namespace ICM42688;
uint8_t cICM42688::ReadReg(uint8_t reg)
{
    _databuf[17]=reg|0x80;
    _pspi->CS_Enable();
    _pspi->ExchangeByte(_databuf+17,_databuf+17,2);
    _pspi->CS_Disable();
    return _databuf[18];
}

uint8_t cICM42688::WriteReg(uint8_t reg, uint8_t data)
{
    _databuf[17]=reg;
    _databuf[18]=data;
    _pspi->CS_Enable();
    _pspi->ExchangeByte(_databuf+17,_databuf+17,2);
    _pspi->CS_Disable();
    return 0;
}

uint8_t cICM42688::UpdateTem(void)
{
    _databuf[0] = 0x1D|0x80;
    _pspi->CS_Enable();
    _databuf[3] = _pspi->ExchangeByte(_databuf,_databuf,3);
    _pspi->CS_Disable();

    /*Check spi communication*/
    if(_databuf[3]!=0){return 0x01;}

    _temperature = 0.00755287f*((_databuf[1]<<8)|(_databuf[2])) + 25.0f;
    return 0;
}

uint8_t cICM42688::UpdateAccel(void)
{
    _databuf[0] = 0x1F|0x80;
    _pspi->CS_Enable();
    _databuf[7] = _pspi->ExchangeByte(_databuf,_databuf,7);
    _pspi->CS_Disable();

    /*Check spi communication*/
    if(_databuf[7]!=0){return 0x01;}

    _accel[0] = _databuf[1]<<8 | _databuf[2];
    _accel[1] = _databuf[3]<<8 | _databuf[4];
    _accel[2] = _databuf[5]<<8 | _databuf[6];
    return 0;
}

uint8_t cICM42688::UpdateGyro(void)
{
    _databuf[0] = 0x25|0x80;
    _pspi->CS_Enable();
    _databuf[7] = _pspi->ExchangeByte(_databuf,_databuf,7);
    _pspi->CS_Disable();

    /*Check spi communication*/
    if(_databuf[7]!=0){return 0x01;}

    _gyro[0] = _databuf[1]<<8 | _databuf[2];
    _gyro[1] = _databuf[3]<<8 | _databuf[4];
    _gyro[2] = _databuf[5]<<8 | _databuf[6];
    return 0;
}

uint8_t cICM42688::UpdateAll(void)
{
    _databuf[0] = 0x1D|0x80;
    _pspi->CS_Enable();
    _databuf[15] = _pspi->ExchangeByte(_databuf,_databuf,15);
    _pspi->CS_Disable();

    /*Check spi communication*/
    if(_databuf[15]!=0){return 0x01;}

    _temperature = 0.00755287f*((_databuf[1]<<8)|(_databuf[2])) + 25.0f;
    _accel[0] = _databuf[3]<<8 | _databuf[4];
    _accel[1] = _databuf[5]<<8 | _databuf[6];
    _accel[2] = _databuf[7]<<8 | _databuf[8];
    _gyro[0] = _databuf[9]<<8 | _databuf[10];
    _gyro[1] = _databuf[11]<<8 | _databuf[12];
    _gyro[2] = _databuf[13]<<8 | _databuf[14];

    return 0;
}

void cICM42688::GetAccel(uint8_t* pdata)
{
    memcpy(pdata,_accel,6);
}

void cICM42688::GetGyro(uint8_t* pdata)
{
    memcpy(pdata,_gyro,6);
}

float cICM42688::GetTem()
{
    return _temperature; 
}

/*A example of configuration*/
// static void IMU_Init(cICM42688* icm42688)
// {
// 	uint8_t buf = 0;
// 	/*Set at Bank0*/
// 	icm42688->WriteReg(0x76,0x00);
// 	/*Soft reset*/
// 	icm42688->WriteReg(0x11,0x01);
//     tx_thread_sleep(5);
// 	/*Read interrput flag to set spi protocol*/
// 	buf = icm42688->ReadReg(0x2D);

// 	/*Set at Bank0*/
// 	icm42688->WriteReg(0x76,0x00);
// 	/*Interrupt pin set*/
// 	icm42688->WriteReg(0x14,0x12);//INT1 INT2 Pulse, Low is available
// 	/*Set gyro*/
// 	icm42688->WriteReg(0x4F,0x06);//2000dps 1KHz
// 	/*Set accel*/
// 	icm42688->WriteReg(0x50,0x06);//16G 1KHz

// 	/*INT_CONFIG0*/
// 	icm42688->WriteReg(0x63,0x00);//Null
// 	/*INT_CONFIG1*/
// 	icm42688->WriteReg(0x64,0x00);//Enable interrupt pin
// 	/*INT_SOURCE0*/
// 	icm42688->WriteReg(0x65,0x08);//DRDY INT1
// 	/*INT_SOURCE1*/
// 	icm42688->WriteReg(0x66,0x00);//Null
// 	/*INT_SOURCE3*/
// 	icm42688->WriteReg(0x68,0x00);//Null
// 	/*INT_SOURCE3*/
// 	icm42688->WriteReg(0x69,0x00);//Null
	
// /*****Anti-Aliasing Filter@536Hz*****/

// 	/*GYRO Anti-Aliasing Filter config*/
// 	/*Set to Bank1*/
// 	icm42688->WriteReg(0x76,0x01);
// 	/*Config anti-aliasing-filter of gyro*/
// 	icm42688->WriteReg(0x0B,0xA0);//Enable AAF and Notch filter
// 	icm42688->WriteReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
// 	icm42688->WriteReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
// 	icm42688->WriteReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
// 	/*ACCEL Anti-Aliasing Filter config*/
// 	/*Set to bank2*/
// 	icm42688->WriteReg(0x76,0x02);
// 	/*Config anti-aliasing-filter of accel*/
// 	icm42688->WriteReg(0x03,0x18);//Enable Notch filter ACCEL_AFF_DELT 12 (default 24)
// 	icm42688->WriteReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
// 	icm42688->WriteReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

// /********User filter********/

// 	/*Set to bank 0*/
// 	icm42688->WriteReg(0x76,0x00);
// 	/*Set filters order*/
// 	icm42688->WriteReg(0x51,0x80);//Temp@20Hz GYRO_UI_FILTER@1nd-order
// 	icm42688->WriteReg(0x53,0x0D);//GYRO_UI_FILTER@1nd-order
// 	/*User filter set*/
// 	icm42688->WriteReg(0x52,0xFF);//GYRO/ACCLE_UI_FILT_BW@0 -3BW=2096.3Hz NBW=2204.6Hz GroupLatancy=0.2ms


// /*****Set to normal mode*****/
//  /*Config mcu gpio interrupt*/
// 	NVIC_EnableIRQ(EXTI15_10_IRQn);
// 	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
// 	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_11);
// 	NVIC_EnableIRQ(EXTI1_IRQn);
// 	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
// 	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

// 	/*Set to bank0*/
// 	icm42688->WriteReg(0x76,0x00);
// 	/*Set power manager*/
// 	icm42688->WriteReg(0x4E,0x0F);//ACC GYRO enable at LowNoise Mode
// }