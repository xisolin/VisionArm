//
// Created by 0209 on 2026/1/10.
//

#include "include/MT6816_STM32.h"

void MT6816_STM32::SpiInit()
{
}

uint16_t MT6816_STM32::SpiTransmitAndRead16Bits(uint16_t txData)
{
    uint16_t rxData = 0;
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 100);
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
    return rxData;
}

