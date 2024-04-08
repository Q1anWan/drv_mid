/*
 * @Description: A orangepi linux instance of SPIA
 * @Author: qianwan
 * @Date: 2024-04-08 00:00:00
 * @LastEditTime: 2024-04-08 00:00:00
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
// Notice : All spi behavior will block threads!!!
#pragma once
#ifndef LIB_SPI_I_
#define LIB_SPI_I_
#include "libspi-a-1.0.h"

#include <cstdint>
#include <cstring>

#include "sys/ioctl.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/spi/spi.h"
#include "linux/spi/spidev.h"

namespace SPI
{
    class cSPI : public SPIA::cSPIA
    {
    protected:
        struct spi_ioc_transfer _tr{};
        int _fd = -1;

    public:
        cSPI() = default;
        cSPI(const char *dev, uint8_t mode, uint8_t bits, uint32_t speed_hz, uint16_t delay_usecs=0)
        {
            _fd = open(dev, O_RDWR);
            if (_fd == -1)
            {
                _init_status = 0;
                return;
            }

            int ret = ioctl(_fd, SPI_IOC_WR_MODE, &mode);
            if (ret == -1)
            {
                close(_fd);
                _init_status = 0;
                return;
            }

            ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
            if (ret == -1)
            {
                close(_fd);
                _init_status = 0;
                return;
            }

            ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
            if (ret == -1)
            {
                close(_fd);
                _init_status = 0;
                return;
            }

            _tr.bits_per_word = bits;
            _tr.speed_hz = speed_hz;
            _tr.delay_usecs = delay_usecs;
            _init_status = 0x02;
        }

        ~cSPI()
        {
            _init_status = 0;
            if (_fd != -1)
            {
                close(_fd);
            }
        }

        void CS_Enable() override
        {;//do nothing
        }

        void CS_Disable() override
        {;//do nothing
        }

        uint8_t ExchangeByte(uint8_t data) override
        {
            uint8_t rx_data=0;
            _tr.tx_buf = (uint64_t)&data;
            _tr.rx_buf = (uint64_t)&rx_data;
            _tr.len = 1;

            int ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_tr);
            if (ret == 1)
            {
                return rx_data;
            }
            return 0;
        }

        uint8_t ExchangeByte(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num) override
        {
            _tr.tx_buf = (uint64_t)pdatatx;
            _tr.rx_buf = (uint64_t)pdatarx;
            _tr.len = num;

            int ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_tr);
            if (ret == 1)
            {
                return 0;
            }
            return 1;
        }

        uint8_t WriteByte(uint8_t *pdata, uint32_t num) override
        {
            _tr.tx_buf = (uint64_t)pdata;
            _tr.rx_buf = 0;
            _tr.len = num;

            int ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_tr);
            if (ret == 1)
            {
                return 0;
            }
            return 1;
        }

        uint8_t ReadByte(uint8_t *pdata, uint32_t num) override
        {
            _tr.tx_buf = 0;
            _tr.rx_buf = (uint64_t)pdata;
            _tr.len = num;

            int ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_tr);
            if (ret == 1)
            {
                return 0;
            }
            return 1;
        }

        uint8_t TransmitDMA(uint8_t *pdata, uint32_t num) override
        {
            return WriteByte(pdata, num);
        }

        uint8_t ReceiveDMA(uint8_t *pdata, uint32_t num) override
        {
            return ReadByte(pdata, num);
        }

        uint8_t TransmitReceiveDMA(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num) override
        {
            return ExchangeByte(pdatatx, pdatarx, num);
        }

    };
}


#endif // LIB_SPI_I_