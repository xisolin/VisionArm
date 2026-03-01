#include "include/MT6816.h"

MT6816::MT6816(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(spi), cs_port_(cs_port), cs_pin_(cs_pin) {}

bool MT6816::Init() {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    GetRawAngle();
    return true;
}

uint16_t MT6816::GetRawAngle() {
    uint8_t tx_03[2] = {0x83, 0x00};
    uint8_t rx_03[2] = {0x00, 0x00};

    uint8_t tx_04[2] = {0x84, 0x00};
    uint8_t rx_04[2] = {0x00, 0x00};

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi_, tx_03, rx_03, 2, 10);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    for(volatile int i=0; i<10; i++);

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi_, tx_04, rx_04, 2, 10);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    uint8_t angle_h = rx_03[1];
    uint8_t angle_l = rx_04[1];

    uint16_t raw_16bit = (angle_h << 8) | angle_l;

    uint16_t angle_14bit = raw_16bit >> 2;

    return angle_14bit;
}

float MT6816::GetAngleDegrees() {
    uint16_t raw = GetRawAngle();
    return (float)raw * 360.0f / 16384.0f;
}
