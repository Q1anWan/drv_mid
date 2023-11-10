/*
 * @Description: A instantiation of IMU-ICM42688
 * @Author: qianwan
 * @Date: 2023-10-30 18:41:47
 * @LastEditTime: 2023-11-10 10:59:23
 * @LastEditors: qianwan
 */
 /******************************************************************************
  * @attention
  * BSD 3-Clause License
  * Copyright (c) 2023, Qianwan.Jin
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************/
 /*Version 1.0*/
 /*Stepper 0.0*/
#pragma once
#ifndef LIB_ICM42688_
#define LIB_ICM42688_
#include "libspi-a-1.0.h"
#include "libimu-a-1.0.h"
#include "string"

namespace ICM42688 {
    class cICM42688 : public IMUA::cIMUA{
    protected:
        SPIA::cSPIA* _pspi;         //SPI handle
        uint8_t _databuf[19]={0};   //Buffer used to communicate with IMU
        int16_t _accel[3];          //Raw acceleration
        int16_t _gyro[3];           //Raw angular velocity
        float _temperature=0.0f;    //Temperature of IMU
        
    public:
        cICM42688(SPIA::cSPIA* pSPI):
        _pspi(pSPI){}
        cICM42688(){}

        uint8_t ReadReg(uint8_t reg);                   //Read a register from IMU
        uint8_t WriteReg(uint8_t reg, uint8_t data);    //Write a register from IMU

        uint8_t UpdateAccel() override; //Read raw acceleration
        uint8_t UpdateGyro() override;  //Read raw angular velocity
        uint8_t UpdateTem() override;   //Read temperature
        uint8_t UpdateAll() override;   //Read accel,gyro,tem
        
        void GetAccel(uint8_t* pdata) override; //Get raw data
        void GetGyro(uint8_t* pdata) override;  //Get raw data
        float GetTem() override;                //Get raw data
    };
}  // namespace ICM42688

#endif