/*
 * @Description: An instance of SPIA
 * @Author: qianwan
 * @Date: 2023-10-30 18:42:55
 * @LastEditTime: 2023-11-08 20:04:24
 * @LastEditors: qianwan
 */
 
 /*Version 1.0*/
 /*Stepper 0.2*/

#pragma once
#ifndef LIB_SPI_I_
#define LIB_SPI_I_
#include <cstdint>
#include "libspi-a-1.0.h"

/***********Config library***********/

// #include "stm32f1xx_ll_spi.h"
// #include "stm32f1xx_ll_gpio.h"
// #if defined (DMA1)
// #include "stm32f1xx_ll_dma.h"
// #endif

// #include "stm32f4xx_ll_spi.h"
// #include "stm32f4xx_ll_gpio.h"
// #if defined (DMA1)
// #include "stm32f4xx_ll_dma.h"
// #endif

 #include "stm32h7xx_ll_spi.h"
 #include "stm32h7xx_ll_gpio.h"
 #if defined (DMA1)
 #include "stm32h7xx_ll_dma.h"
 #endif

/***********************************/

namespace SPI
{
    /**
     * @brief  Basic spi driver instance
     */
    class cSPIBasic : public SPIA::cSPIA{
    protected:
        SPI_TypeDef* _pspi;
        uint32_t _maxwaitcnt = 1000;
        GPIO_TypeDef* _gpio_port_cs;
        uint32_t _gpio_pin_cs;

        inline void SPIEnable(uint32_t size)
        {
#ifdef SPI_CR1_CSTART
            LL_SPI_SetTransferSize(_pspi,size);
#endif
            LL_SPI_Enable(_pspi);
#ifdef SPI_CR1_CSTART
            LL_SPI_StartMasterTransfer(_pspi);
#endif
        }

        inline void SPIDisable()
        {
#ifdef SPI_CR1_CSTART
            LL_SPI_ClearFlag_EOT(SPI1);
            LL_SPI_ClearFlag_TXTF(SPI1);
#endif
            LL_SPI_Disable(_pspi);
        }

    public:
        cSPIBasic(
            SPI_TypeDef* pSPI,                                  /*SPI handel*/
            GPIO_TypeDef* GPIO_PORT_CS,
            uint32_t GPIO_PIN_CS,
            uint32_t maxBytetime
        )
        :_maxwaitcnt(maxBytetime)
        {
            bool check_pointer = (pSPI == nullptr) || (GPIO_PORT_CS == nullptr);
            /*Check pointers*/
            if(check_pointer)
            {
                SPIA::cSPIA::_init_status = 0x00;
                return;
            }
            _pspi = pSPI;
            _gpio_port_cs = GPIO_PORT_CS;
            _gpio_pin_cs = GPIO_PIN_CS;
            SPIA::cSPIA::_init_status = 0x01;
        }

        inline bool SPICheckTXE()
        {
#ifdef SPI_CR1_CSTART
            return LL_SPI_IsActiveFlag_TXP(_pspi);
#else
            return LL_SPI_IsActiveFlag_TXE(_pspi);
#endif
        }
        inline bool SPICheckRXNE()
        {
#ifdef SPI_CR1_CSTART
            return LL_SPI_IsActiveFlag_RXP(_pspi);
#else
            return LL_SPI_IsActiveFlag_RXNE(_pspi);
#endif
        }
        inline void CS_Enable()override{LL_GPIO_ResetOutputPin(_gpio_port_cs,_gpio_pin_cs);}
        inline void CS_Disable()override{LL_GPIO_SetOutputPin(_gpio_port_cs,_gpio_pin_cs);}

        uint8_t ExchangeByte(uint8_t data) override
        {
            /*Set an over time*/
            volatile uint32_t wait_cnt = 0;
            uint8_t tmp;

            /*Enable SPI and start transform*/
            SPIEnable(1);

            /*Wait until TX buff is empty*/
            while(SPICheckTXE()==0){
                if (wait_cnt++ > _maxwaitcnt)
                {
                    LL_SPI_Disable(_pspi);
                    LL_SPI_ClearFlag_OVR(_pspi);
                    return 0x00;
                }
            }

            /*Transmit data*/
            LL_SPI_TransmitData8(_pspi,data);

            /*Block until transmission is ready*/
            while(SPICheckRXNE()==0){
                if (wait_cnt++ > _maxwaitcnt)
                {
                    SPIDisable();
                    return 0x00;
                }
            }

            tmp = LL_SPI_ReceiveData8(_pspi);

            /*Disable SPI*/
            SPIDisable();
            return tmp;
        }

        uint8_t ExchangeByte(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num) override
        {
            /*Set an over time*/
            volatile uint32_t wait_cnt = 0;
            uint32_t tx_ref_num = num;
            uint32_t rx_ref_num = num;
#ifndef SPI_CR1_CSTART
            volatile uint8_t tx_allowed = 1;
#endif
            /*Check error*/
            if ((pdatatx == nullptr) || (pdatarx == nullptr) || (num == 0U))
            {
                return 0x01;
            }
            /*Enable SPI and start transform*/
            SPIEnable(num);

            while((tx_ref_num>0U)||(rx_ref_num>0U)){
#ifdef SPI_CR1_CSTART /*Has fif0, TX&RX no need to execute in order*/
                if((SPICheckTXE()==1)&&(tx_ref_num>0U)){
                    LL_SPI_TransmitData8(_pspi,*pdatatx);
                    pdatatx += sizeof(uint8_t);
                    tx_ref_num--;
                }

                if((SPICheckRXNE()==1)&&(rx_ref_num>0U)){
                    *pdatarx = LL_SPI_ReceiveData8(_pspi);
                    pdatarx += sizeof(uint8_t);
                    rx_ref_num--;
                    wait_cnt = 0;
                }
#else   /*No fif0, TX&RX need to excute in order*/
                if((SPICheckTXE()==1)&&(tx_ref_num>0U)&&(tx_allowed==1)){
                LL_SPI_TransmitData8(_pspi,*pdatatx);
                pdatatx += sizeof(uint8_t);
                tx_ref_num--;
                tx_allowed = 0;
            }

            if((SPICheckRXNE()==1)&&(rx_ref_num>0U)){
                *pdatarx = LL_SPI_ReceiveData8(_pspi);
                pdatarx += sizeof(uint8_t);
                rx_ref_num--;
                tx_allowed = 1;
                wait_cnt = 0;
            }
#endif
                if(++wait_cnt>_maxwaitcnt){
                    SPIDisable();
                    return 0x01;
                }
            }

            SPIDisable();
            return 0;
        }

        uint8_t WriteByte(uint8_t *pdata, uint32_t num) override
        {
            /*Set an over time*/
            volatile uint32_t wait_cnt = 0;
            uint32_t tx_ref_num = num;

            /*Check error*/
            if ((pdata == nullptr) || (num == 0U))
            {
                return 0x01;
            }
            /*Enable SPI and start transform*/
            SPIEnable(num);

#ifdef SPI_CR1_CSTART
            while (tx_ref_num > 0U)
            {
                /* Wait until TXE flag is set to send data */
                if(SPICheckTXE()==1){
                    if((tx_ref_num>3U)&&(LL_SPI_GetFIFOThreshold(_pspi)>LL_SPI_FIFO_TH_03DATA)){
                        LL_SPI_TransmitData32(_pspi,*((const uint32_t*)pdata));
                        pdata += sizeof(uint32_t);
                        tx_ref_num -= 4U;
                    }
                    else if((tx_ref_num>1U)&&(LL_SPI_GetFIFOThreshold(_pspi)>LL_SPI_FIFO_TH_01DATA)){
                        LL_SPI_TransmitData16(_pspi,*((const uint16_t*)pdata));
                        pdata += sizeof(uint16_t);
                        tx_ref_num -= 2U;
                    }
                    else{
                        LL_SPI_TransmitData8(_pspi,*pdata);
                        pdata += sizeof(uint8_t);
                        tx_ref_num--;
                    }
                    wait_cnt = 0;
                }
                else if(++wait_cnt>_maxwaitcnt)
                {
                    SPIDisable();
                    return 0x01;
                }
            }

            /*Wait for the last transmission*/
            while(LL_SPI_IsActiveFlag_EOT(_pspi)==0){
                if(++wait_cnt>_maxwaitcnt)
                {
                    SPIDisable();
                    return 0x01;
                }
            }
#else
            while (tx_ref_num > 0U)
        {
          /* Wait until TXE flag is set to send data */
          if(SPICheckTXE()==1){
            LL_SPI_TransmitData8(_pspi,*pdata);
            pdata += sizeof(uint8_t);
            tx_ref_num--;
            wait_cnt = 0;
          }
          else if(++wait_cnt>_maxwaitcnt)
          {
                SPIDisable();
                return 0x01;
          }
        }
#endif

            SPIDisable();
            return 0;
        }

        uint8_t ReadByte(uint8_t *pdata, uint32_t num) override
        {
            return ExchangeByte(pdata,pdata,num);

//        /*Set an over time*/
//        volatile uint32_t wait_cnt = 0;
//        uint32_t rx_ref_num = num;
//
//        /*Check error*/
//        if ((pdata == nullptr) || (num == 0U))
//        {
//            return 0x01;
//        }
//
//        while(rx_ref_num>0U){
//            /* Wait until RXNE flag is set to send data */
//            if(SPICheckRXNE()==1){
//                *pdata = LL_SPI_ReceiveData8(_pspi);
//                pdata += sizeof(uint8_t);
//                rx_ref_num--;
//                wait_cnt = 0;
//            }
//            else if(++wait_cnt>_maxwaitcnt)
//            {
//                return 0x01;
//            }
//        }
//
//        return 0;
        }
    };

#if defined (DMA1)

    typedef uint32_t (FUNC_DMA_FLAG_CHECK)(DMA_TypeDef *DMAx);
    typedef void (FUNC_DMA_FLAG_OPERATE)(DMA_TypeDef *DMAx);
    typedef void (FUNC_DMA_CHN_SET)(DMA_TypeDef *DMAx, uint32_t CHNx);

#define SPI_DMA_T template< \
    FUNC_DMA_FLAG_CHECK funcDMA_CheckFlag_TX_TC, FUNC_DMA_FLAG_OPERATE funcDMA_ClearFlag_TX_TC, FUNC_DMA_FLAG_OPERATE funcDMA_ClearFlag_TX_TE, \
    FUNC_DMA_FLAG_CHECK funcDMA_CheckFlag_RX_TC, FUNC_DMA_FLAG_OPERATE funcDMA_ClearFlag_RX_TC, FUNC_DMA_FLAG_OPERATE funcDMA_ClearFlag_RX_TE, \
    FUNC_DMA_CHN_SET    funcDMA_EnableCHN,       FUNC_DMA_CHN_SET funcDMA_DisableCHN>
#define SPI_DMA_P funcDMA_CheckFlag_TX_TC,funcDMA_ClearFlag_TX_TC,funcDMA_ClearFlag_TX_TE,funcDMA_CheckFlag_RX_TC,funcDMA_ClearFlag_RX_TC,funcDMA_ClearFlag_RX_TE,funcDMA_EnableCHN,funcDMA_DisableCHN

    /**
    * @brief  Advance spi instance including DMA
    * @note   Use template
    */
    SPI_DMA_T class cSPI : public cSPIBasic {
        protected:
        DMA_TypeDef *_pdma_tx = nullptr;
        uint32_t     _dma_tx_chn = 0;
        DMA_TypeDef *_pdma_rx = nullptr;
        uint32_t     _dma_rx_chn = 0;

        inline bool SPICheckIT_EOT()
        {
    #ifdef SPI_CR1_CSTART
            return LL_SPI_IsActiveFlag_EOT(_pspi);
    #else
            return LL_SPI_IsActiveFlag_RXNE(_pspi);
    #endif
        }

        inline void SPIClearIT_EOT()
        {
#ifdef SPI_CR1_CSTART
            return LL_SPI_ClearFlag_EOT(_pspi);
#endif
        }

        inline void SPIEnableIT_EOT()
        {
    #ifdef SPI_CR1_CSTART
            LL_SPI_EnableIT_EOT(_pspi);
    #else
            LL_SPI_EnableIT_RXNE(_pspi);
    #endif
        }

        inline void SPIDisableIT_EOT()
        {
    #ifdef SPI_CR1_CSTART
            LL_SPI_DisableIT_EOT(_pspi);
    #else
            LL_SPI_DisableIT_RXNE(_pspi);
    #endif
        }

        inline bool DMATXCheckTC()
        {return funcDMA_CheckFlag_TX_TC(_pdma_tx);}
        inline void DMATXClearTC()
        {funcDMA_ClearFlag_TX_TC(_pdma_tx);}
        inline void DMATXClearTE()
        {funcDMA_ClearFlag_TX_TE(_pdma_tx);}


        inline bool DMARXCheckTC()
        {return funcDMA_CheckFlag_RX_TC(_pdma_rx);}
        inline void DMARXClearTC()
        {funcDMA_ClearFlag_RX_TC(_pdma_rx);}
        inline void DMARXClearTE()
        {funcDMA_ClearFlag_RX_TE(_pdma_rx);}


        inline void DMATXEnableCHN()
        {funcDMA_EnableCHN(_pdma_tx,_dma_tx_chn);}
        inline void DMATXDisableCHN()
        {funcDMA_DisableCHN(_pdma_tx,_dma_tx_chn);}
        inline void DMARXEnableCHN()
        {funcDMA_EnableCHN(_pdma_rx,_dma_rx_chn);}
        inline void DMARXDisableCHN()
        {funcDMA_DisableCHN(_pdma_rx,_dma_rx_chn);}

        public:
        cSPI(
            SPI_TypeDef* pSPI,                                  /*SPI handel*/
            GPIO_TypeDef* GPIO_PORT_CS,
            uint32_t GPIO_PIN_CS,
            uint32_t maxBytetime,
            DMA_TypeDef* pDMA_TX,
            uint32_t     DMA_TX_CHN,
            DMA_TypeDef* pDMA_RX,
            uint32_t     DMA_RX_CHN
            ):
        cSPIBasic(pSPI,GPIO_PORT_CS,GPIO_PIN_CS,maxBytetime),
        _pdma_tx(pDMA_TX),_dma_tx_chn(DMA_TX_CHN),_pdma_rx(pDMA_RX),_dma_rx_chn(DMA_RX_CHN)
        {
            if(
                (pSPI==nullptr)||(GPIO_PORT_CS==nullptr)||(pDMA_RX==nullptr)||(pDMA_TX==nullptr)||                              \
                (funcDMA_CheckFlag_TX_TC==nullptr)||(funcDMA_ClearFlag_TX_TC==nullptr)||(funcDMA_ClearFlag_TX_TE==nullptr)||    \
                (funcDMA_CheckFlag_RX_TC==nullptr)||(funcDMA_ClearFlag_RX_TC==nullptr)||(funcDMA_ClearFlag_RX_TE==nullptr)||    \
                (funcDMA_EnableCHN==nullptr)||(funcDMA_DisableCHN==nullptr)
                )
            {
                return;
            }

            cSPI::cSPIA::_init_status = 2;
        }

        uint8_t TransmitDMA(uint8_t *pdata, uint32_t num) override;
        uint8_t ReceiveDMA(uint8_t *pdata, uint32_t num) override;
        uint8_t TransmitReceiveDMA(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num) override;

        uint8_t TransmitDMAIRQ() override;
        uint8_t ReceiveDMAIRQ() override;
        uint8_t SPIIRQ() override;
    };

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::TransmitDMA(uint8_t *pdata, uint32_t num)
    {
        /*Make sure not cause to memory overrun*/
        if((pdata==nullptr)||(num==0U))
        {
            return 0x01;
        }

        /*Config DMA first*/
        DMATXDisableCHN();
#ifdef SPI_CR1_CSTART
        LL_DMA_ConfigAddresses(_pdma_tx,_dma_tx_chn,(uint32_t)pdata,LL_SPI_DMA_GetTxRegAddr(_pspi),LL_DMA_GetDataTransferDirection(_pdma_tx, _dma_tx_chn));
#else
        LL_DMA_ConfigAddresses(_pdma_tx,_dma_tx_chn,(uint32_t)pdata,LL_SPI_DMA_GetRegAddr(_pspi),LL_DMA_GetDataTransferDirection(_pdma_tx, _dma_tx_chn));
#endif
        LL_DMA_SetDataLength(_pdma_tx,_dma_tx_chn,num);
        DMATXClearTC();
        DMATXClearTE();
        LL_DMA_EnableIT_TC(_pdma_tx,_dma_tx_chn);
        DMATXEnableCHN();

        /*Then config and enable SPI*/
        LL_SPI_EnableDMAReq_TX(_pspi);
        SPIEnable(num);

        return 0;
    }

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::ReceiveDMA(uint8_t *pdata, uint32_t num)
    {
        /*Make sure not cause to memory overrun*/
        if((pdata==nullptr)||(num==0U))
        {
            return 0x01;
        }

        /*Config DMA first*/
        DMARXDisableCHN();
#ifdef SPI_CR1_CSTART
        LL_DMA_ConfigAddresses(_pdma_rx,_dma_rx_chn,LL_SPI_DMA_GetRxRegAddr(_pspi),(uint32_t)pdata,LL_DMA_GetDataTransferDirection(_pdma_rx, _dma_rx_chn));
#else
        LL_DMA_ConfigAddresses(_pdma_rx,_dma_rx_chn,LL_SPI_DMA_GetRegAddr(_pspi),(uint32_t)pdata,LL_DMA_GetDataTransferDirection(_pdma_rx, _dma_rx_chn));
#endif
        LL_DMA_SetDataLength(_pdma_rx,_dma_rx_chn,num);
        DMARXClearTC();
        DMARXClearTE();
        LL_DMA_EnableIT_TC(_pdma_rx,_dma_rx_chn);
        DMARXEnableCHN();

        /*Then config and enable SPI*/
        LL_SPI_EnableDMAReq_RX(_pspi);

        /*SPI Enable shouldn't be set in dma_rx in case transform number is set after spi enabled*/
        return 0;
    }

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::TransmitReceiveDMA(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num)
    {
        /*Make sure not cause to memory overrun*/
        if((pdatatx==nullptr)||(pdatarx==nullptr)||(num==0U))
        {
            return 0x01;
        }

        /*Config DMA first*/
        DMARXDisableCHN();
        DMATXDisableCHN();

#ifdef SPI_CR1_CSTART
        LL_DMA_ConfigAddresses(_pdma_rx,_dma_rx_chn,LL_SPI_DMA_GetRxRegAddr(_pspi),(uint32_t)pdatarx,LL_DMA_GetDataTransferDirection(_pdma_rx, _dma_rx_chn));
        LL_DMA_ConfigAddresses(_pdma_tx,_dma_tx_chn,(uint32_t)pdatatx,LL_SPI_DMA_GetTxRegAddr(_pspi),LL_DMA_GetDataTransferDirection(_pdma_tx, _dma_tx_chn));
#else
        LL_DMA_ConfigAddresses(_pdma_rx,_dma_rx_chn,LL_SPI_DMA_GetRegAddr(_pspi),(uint32_t)pdatarx,LL_DMA_GetDataTransferDirection(_pdma_rx, _dma_rx_chn));
        LL_DMA_ConfigAddresses(_pdma_tx,_dma_tx_chn,(uint32_t)pdatatx,LL_SPI_DMA_GetRegAddr(_pspi),LL_DMA_GetDataTransferDirection(_pdma_tx, _dma_tx_chn));
#endif
        LL_DMA_SetDataLength(_pdma_rx,_dma_rx_chn,num);
        LL_DMA_SetDataLength(_pdma_tx,_dma_tx_chn,num);
        DMARXClearTC();
        DMARXClearTE();
        DMATXClearTC();
        DMATXClearTE();
        LL_DMA_EnableIT_TC(_pdma_rx,_dma_rx_chn);
        LL_DMA_EnableIT_TC(_pdma_tx,_dma_tx_chn);
        DMARXEnableCHN();
        DMATXEnableCHN();

        /*Then config and enable SPI*/
        LL_SPI_EnableDMAReq_RX(_pspi);
        LL_SPI_EnableDMAReq_TX(_pspi);
        SPIEnable(num);

        return 0;
    }

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::TransmitDMAIRQ()
    {
        /*Make sure really TC IRQ is coming*/
        if(DMATXCheckTC()==0)
        {return 0x01;}

        /*We will disable SPI in EOT interrupt to make sure last transmission is finish.*/
        SPIEnableIT_EOT();

        /*Disable DMA*/
        LL_SPI_DisableDMAReq_TX(_pspi);
        DMATXDisableCHN();
        LL_DMA_DisableIT_TC(_pdma_tx,_dma_tx_chn);
        DMATXClearTC();
        DMATXClearTE();
        return 0;
    }

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::ReceiveDMAIRQ()
    {
        /*Make sure really TC IRQ is coming*/
        if(DMARXCheckTC()==0)
        {return 0x01;}

        /*Disable DMA*/
        LL_SPI_DisableDMAReq_RX(_pspi);
        DMARXDisableCHN();
        LL_DMA_DisableIT_TC(_pdma_rx,_dma_rx_chn);
        DMARXClearTC();
        DMARXClearTE();

        return 0;
    }

    SPI_DMA_T uint8_t cSPI<SPI_DMA_P>::SPIIRQ()
    {
        if(SPICheckIT_EOT()==0)
        {
            return 0X01;
        }
        SPIClearIT_EOT();
        SPIDisableIT_EOT();
        SPIDisable();
        return 0;
    }

#endif
}
#endif