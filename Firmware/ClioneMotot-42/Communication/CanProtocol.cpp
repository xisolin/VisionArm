#include "CanProtocol.h"

/**
 * @brief 构造函数
 * @param hcan: STM32 HAL库的 CAN 句柄
 * @param targetId: 默认发送的目标 CAN ID
 * @param isExtended: 【新增】是否使用扩展帧 (true=29位, false=11位)
 */
CanProtocol::CanProtocol(CAN_HandleTypeDef* hcan, uint32_t targetId, bool isExtended)
	: ProtocolBase(1024),
	  _hcan(hcan),
	  _targetId(targetId),
	  _isExtended(isExtended) // 记录下来
{
}


bool CanProtocol::RawSend(const uint8_t* pData, size_t len)
{
    if (_hcan == nullptr) return false;

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    if (_isExtended)
    {
        TxHeader.IDE = CAN_ID_EXT;       // 告诉硬件：我要发扩展帧
        TxHeader.ExtId = _targetId;      // ID 填在 ExtId 字段 (29 bit)
        TxHeader.StdId = 0;              // 标准 ID 字段清零 (安全起见)
    }
    else
    {
        TxHeader.IDE = CAN_ID_STD;       // 告诉硬件：我要发标准帧
        TxHeader.StdId = _targetId;      // ID 填在 StdId 字段 (11 bit)
        TxHeader.ExtId = 0;
    }

    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

    size_t remaining = len;
    size_t offset = 0;

    while (remaining > 0)
    {
        uint8_t chunkLen = (remaining > 8) ? 8 : remaining;
        TxHeader.DLC = chunkLen;

        uint32_t retry = 0;
        const uint32_t MAX_RETRY = 50000;

        while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
        {
            retry++;
            if (retry > MAX_RETRY) {
                HandleHardwareError(CommError::TIMEOUT);
                return false; // 硬件忙，直接退出
            }
        }

        if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, (uint8_t*)(pData + offset), &TxMailbox) != HAL_OK)
        {
            HandleHardwareError(CommError::HARDWARE_FAILURE);
            return false;
        }

        remaining -= chunkLen;
        offset += chunkLen;
    }

    return true;
}


bool CanProtocol::Init(uint32_t filterBank, uint32_t fifo)
{
    if (_hcan == nullptr) return false;
    _rxFifo = fifo;


    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = filterBank;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = fifo;

    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;


    if (HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
    {
        return false;
    }

    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        return false;
    }

    uint32_t it_flag = (fifo == CAN_RX_FIFO0) ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;

    if (HAL_CAN_ActivateNotification(_hcan, it_flag) != HAL_OK)
    {
        return false;
    }

    return true;
}


void CanProtocol::OnRxIrq()
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, _rxFifo) > 0)
    {
        if (HAL_CAN_GetRxMessage(_hcan, _rxFifo, &RxHeader, RxData) == HAL_OK)
        {
            this->WriteToRingBuffer(RxData, RxHeader.DLC);
        }
    }
}
