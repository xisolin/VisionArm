#include "SerialProtocol.h"

/**
 * @brief 初始化串口工作模式
 */
void SerialProtocol::Init(SerialMode Mode)
{
    switch (Mode)
    {
    case IT:
        HAL_UARTEx_ReceiveToIdle_IT(SerialHuart, rxBuffer.data(), rxBuffer.size());
        break;
    case DMA:
        HAL_UARTEx_ReceiveToIdle_DMA(SerialHuart, rxBuffer.data(), rxBuffer.size());
        __HAL_UART_ENABLE_IT(SerialHuart, UART_IT_IDLE);
        break;
    default:
        break;
    }
}


/**
 * @brief 初始化 RS485 控制引脚
 */
void SerialProtocol::InitRS485(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    Serial485GPIO = GPIOx;
    Serial485Pin = GPIO_Pin;
    SetRxMode();
    // HAL_UART_RegisterCallback(SerialHuart, HAL_UART_TX_COMPLETE_CB_ID, RS485_Transmit_Cplt);
    RestartRx();
}

void SerialProtocol::SetRxMode()
{
    HAL_GPIO_WritePin(Serial485GPIO, Serial485Pin, GPIO_PIN_RESET);
}

void SerialProtocol::RestartRx()
{
    __HAL_UART_CLEAR_OREFLAG(SerialHuart);
    if (HAL_UARTEx_ReceiveToIdle_DMA(SerialHuart, rxBuffer.data(), rxBuffer.size()) != HAL_OK) {
        // 如果重启失败，通常是因为硬件正忙，可以尝试强制重置或记录错误
        HAL_UART_AbortReceive(SerialHuart);
        HAL_UARTEx_ReceiveToIdle_DMA(SerialHuart, rxBuffer.data(), rxBuffer.size());
    }
    __HAL_UART_ENABLE_IT(SerialHuart, UART_IT_IDLE);
}

void SerialProtocol::OnCommFailure(CommError err)
{
    if (err == CommError::HARDWARE_FAILURE)
    {
        RestartRx();
    }
}

/**
 * @brief 核心发送函数 (重写父类虚函数)
 * ProtocolBase 打包好数据后，会调这个函数发出去
 */
bool SerialProtocol::RawSend(const uint8_t* pData, size_t len)
{
    if (len == 0 || pData == nullptr) return false;

    RS485Guard guard(SerialHuart, Serial485GPIO, Serial485Pin);

    HAL_StatusTypeDef status =HAL_UART_Transmit(SerialHuart, const_cast<uint8_t*>(pData), len, 50);

    if (status != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
* @brief  发送一个字符
*/
void SerialProtocol::SendByte(uint8_t Byte)
{
    RawSend(&Byte, 1);
}

/**
* @brief  发送N个字符
* 带485
*/
void SerialProtocol::SendBytes(const uint8_t* pData, size_t Size)
{
    RawSend(pData, Size);
}


/**
 * @brief  发送 C 风格字符串（以 '\0' 结尾）  485风格
 * @param  string  指向以 '\0' 结尾的字符串指针
 */
void SerialProtocol::SendString(const char* string)
{
    SendString(std::string(string));
}


/**
 * @brief  发送 C++ std::string 数据   485方式
 * @param  string  要发送的 std::string 对象
 */
void SerialProtocol::SendString(const std::string& string)
{

    RawSend(reinterpret_cast<const uint8_t*>(string.data()),string.size());
}


/**
 * @brief 发送一个数字
 */
void SerialProtocol::SendNumber(int number)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", number); // 使用 snprintf
    SendString(std::string(buf));
}


/**
 * @brief 发送一个浮点数字，可指定小数位数  485
 *
 * @param number    要发送的浮点数
 * @param precision 要保留的小数位数
 */
void SerialProtocol::SendNumber(float number, int precision)
{
    char buf[32];
    snprintf(buf, sizeof(buf),"%.*f", precision, number);
    SendString(buf);
}


/**
* @brief  printf重定向
*/
int SerialProtocol::fputc(int ch, FILE* f)
{
    SendByte(ch);
    return ch;
}

/**
* @brief  重新定义prinft，只能用于一个串口
*/
void SerialProtocol::Printf(const char* format, ...)
{
    char buf[128]; // 临时缓冲区

    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len > 0)
    {
        SendBytes((uint8_t*)buf, len);
    }
}

/**
 * @brief  DMA 接收专用处理函数
 * @param  len: HAL 库回调传进来的接收长度 (Size)
 * @note   这个函数会自动使用内部的 rxBuffer
 */
void SerialProtocol::HandleRxData(uint16_t len)
{
    HandleRxData(this->rxBuffer.data(), len);
}

/**
 * @brief 接收数据入口
 * 在 main.c 的 HAL_UART_RxCpltCallback (或 EventCallback) 中调用
 */
void SerialProtocol::HandleRxData(uint8_t* buf, uint16_t len)
{
    this->WriteToRingBuffer(buf, len);

    RestartRx();
}


/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart3) // 判断对应串口
    {
        Serial.HandleRxData(Size);
        Serial.RestartRx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        Serial.RestartRx();
    }
}
*/

