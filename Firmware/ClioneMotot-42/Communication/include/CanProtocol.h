//
// Created by MAx_cos on 2026/2/18.
//

#ifndef CANPROTOCOL_H
#define CANPROTOCOL_H

#pragma once
#include "ProtocolBase.h"

class CanProtocol : public ProtocolBase{
public:
	CanProtocol(CAN_HandleTypeDef* hcan, uint32_t targetId, bool isExtended = true);
	void SetTargetId(uint32_t id) { _targetId = id; }
	bool Init(uint32_t filterBank = 0, uint32_t fifo = CAN_RX_FIFO0);
	void OnRxIrq();
protected:
	bool RawSend(const uint8_t* pData, size_t len) override;
private:
	CAN_HandleTypeDef* _hcan;
	uint32_t _targetId;
	bool _isExtended;
	uint32_t _rxFifo;
};



#endif //CANPROTOCOL_H
