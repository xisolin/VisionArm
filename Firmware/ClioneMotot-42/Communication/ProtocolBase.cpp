#include "ProtocolBase.h"

/**
 * @brief  构造函数：初始化协议处理缓冲区
 * @param  bufferSize: 内部环形缓冲区的大小（字节）
 * @note   初始化读写指针为 0，并分配内存空间
 */
ProtocolBase::ProtocolBase(size_t bufferSize)
    : ProcBuffer(bufferSize, 0), rxWriteIndex(0), rxReadIndex(0)
{
}

/**
 * @brief  计算 Modbus CRC16 校验码
 * @param  data: 输入的数据指针
 * @param  len : 数据长度（字节数）
 * @retval 16 位 CRC 校验结果
 *
 * 算法说明：
 * - 初始值：0xFFFF
 * - 多项式：0xA001 (对应 x^16 + x^15 + x^2 + 1)
 * - 结果：低字节在前，高字节在后（Modbus 标准）
 *
 * 特点：
 * - 能检测出单比特错误、突发错误等常见通信错误
 * - 常用于 Modbus RTU、RS485、工业通信协议
 */
uint16_t ProtocolBase::CRC16_Modbus(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  计算 Modbus CRC16 校验码 (环形缓冲区零拷贝版本)
 * @param  startIdx : 校验起始位置 (在 ProcBuffer 中的索引)
 * @param  len      : 需要校验的逻辑数据长度
 * @retval 16位 CRC 校验结果
 *
 * 核心优势 (Zero-Copy Optimization)：
 * - **零拷贝**：直接在环形缓冲区 (RingBuffer) 上滑动计算，无需将数据 `memcpy` 出来。
 * - **自动回绕**：通过 `(startIdx + k) % Size` 逻辑，自动处理数据跨越缓冲区尾部并折返到头部的情况。
 *
 * 实现原理：
 * - 逻辑上把环形缓冲区看作连续数据，物理上通过取模运算访问非连续内存。
 * - 避免了为了校验数据而进行的内存分配和数据搬运，极大提高了接收处理效率。
 */
uint16_t ProtocolBase::CRC16_Modbus_Ring(size_t startIdx, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t k = 0; k < len; k++)
    {
        uint8_t byte = ProcBuffer[(startIdx + k) % ProcBuffer.size()];

        crc ^= byte;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void ProtocolBase::SafeRawSend(const uint8_t* pData, size_t len)
{
    if (!RawSend(pData, len))
    {
        HandleHardwareError(CommError::HARDWARE_FAILURE);
    }
    else
    {
        lastActivityTick = HAL_GetTick();
    }
}

/**
 * @brief  向环形缓冲区写入原始数据 (工业级优化版)
 * @param  data: 待写入数据的起始地址
 * @param  len : 数据长度
 */
void ProtocolBase::WriteToRingBuffer(const uint8_t* data, size_t len)
{
    if (len == 0 || data == nullptr) return;

    size_t bufferSize = ProcBuffer.size();

    // 1. 溢出检查：计算当前实际剩余空间
    uint16_t currentDataSize = RingBuffer_Size();
    uint16_t freeSpace = (bufferSize > currentDataSize) ? (bufferSize - currentDataSize - 1) : 0;

    if (len > freeSpace) {
        HandleHardwareError(CommError::BUFFER_OVERFLOW);
        return;
    }

    // 2. 计算从当前写位置到缓冲区物理末尾的可用连续空间
    size_t bytesToEnd = bufferSize - rxWriteIndex;

    if (len <= bytesToEnd) {
        // 情况 A: 物理内存连续，直接一次拷贝
        std::memcpy(&ProcBuffer[rxWriteIndex], data, len);
        // 使用取模保证循环，如果 bufferSize 是 2 的幂，编译器会自动优化为位运算
        rxWriteIndex = (rxWriteIndex + len) % bufferSize;
    } else {
        // 情况 B: 物理内存不连续，需要绕回头部，分两段拷贝
        // 第一段：填充到缓冲区末尾
        std::memcpy(&ProcBuffer[rxWriteIndex], data, bytesToEnd);
        // 第二段：从缓冲区头部开始填充剩余部分
        size_t remainingLen = len - bytesToEnd;
        std::memcpy(&ProcBuffer[0], &data[bytesToEnd], remainingLen);

        rxWriteIndex = remainingLen; // 更新索引
    }

    // 3. 更新时间戳：记录最后一次接收到物理字节的时间
    lastByteReceivedTick = HAL_GetTick();
}


/**
 * @brief  在环形缓冲区中查找帧头 (0xAA 0x55)
 * @param  start: 搜索起始位置（通常是 rxReadIndex）
 * @param  size : 要搜索的字节范围
 * @retval 找到帧头时返回帧头第一个字节的索引，未找到返回 size_t(-1)
 *
 * 原理：
 * - 遍历指定范围，检查连续两个字节是否匹配 HEAD1 和 HEAD2。
 * - 自动处理环形缓冲区的索引越界回绕。
 */
size_t ProtocolBase::FindFrameHeader(size_t start, size_t size)
{
    for (size_t i = start; i < start + size - 1; ++i)
    {
        if (ProcBuffer[i % ProcBuffer.size()] == FRAME_HEADER1 &&
            ProcBuffer[(i + 1) % ProcBuffer.size()] == FRAME_HEADER2)
        {
            return i;
        }
    }
    return size_t(-1);
}

/**
 * @brief  获取环形缓冲区中当前有效数据的长度
 * @retval 缓冲区中的数据字节数
 * @note   处理了写指针小于读指针（已回绕）的情况。
 */
uint16_t ProtocolBase::RingBuffer_Size()
{
    if (rxWriteIndex >= rxReadIndex)
    {
        return rxWriteIndex - rxReadIndex;
    }
    else
    {
        return ProcBuffer.size() - (rxReadIndex - rxWriteIndex);
    }
}


/**
 * @brief  [核心解析引擎] 从环形缓冲区提取一帧完整数据
 * * @details 该函数是全协议栈唯一的解析逻辑点，负责处理帧同步、变长解析、CRC 校验及数据提取。
 * 流程如下：
 * 1. 检索同步头 (HEAD1, HEAD2)；
 * 2. 预读长度字节，判断总包数据量是否已完全接收；
 * 3. [零拷贝校验] 直接在环形缓冲区上滑动计算 CRC16-Modbus 并比对；
 * 4. 验证帧尾 (TAIL1, TAIL2)；
 * 5. 将有效负载 (Payload) 拷贝至外部缓冲区，并更新读指针。
 * * @param  pOut     [输出] 指向接收目的地的指针。解析成功的有效载荷将存入此处。
 * @param  maxLen   [输入] 外部缓冲区的最大容量。若 Payload 超过此长度，将发生截断以保护内存。
 * * @return size_t   返回解析出的 Payload 实际字节数：
 * - 0:  当前未发现完整帧、校验失败或正在等待数据补齐。
 * - >0: 成功解析出一帧，且已自动从 RingBuffer 中移除该数据包。
 * * @note
 * - 成功解析后会自动更新 @ref lastActivityTick，可用于通讯心跳监测。
 * - 此函数不涉及任何 malloc/new 操作，适用于高频实时任务（如 FreeRTOS Task）。
 * - 帧结构: [HEAD1][HEAD2][LEN][PAYLOAD...][CRC_L][CRC_H][TAIL1][TAIL2]
 */
size_t ProtocolBase::ReceiveRaw(uint8_t* pOut, size_t maxLen)
{
    while (RingBuffer_Size() >= kMinFrameLength)
    {
        size_t frameStart = FindFrameHeader(rxReadIndex, RingBuffer_Size());
        if (frameStart == (size_t)-1) {
            rxReadIndex = (rxReadIndex + 1) % ProcBuffer.size();
            return 0;
        }

        size_t lenIdx = (frameStart + 2) % ProcBuffer.size();
        uint8_t payloadSize = ProcBuffer[lenIdx];
        size_t totalFrameLength = payloadSize + 7; // 头2+长1+CRC2+尾2

        if (RingBuffer_Size() < totalFrameLength) return 0;

        if (CRC16_Modbus_Ring(lenIdx, payloadSize + 1) !=
            (ProcBuffer[(lenIdx + 1 + payloadSize) % ProcBuffer.size()] |
            (ProcBuffer[(lenIdx + 2 + payloadSize) % ProcBuffer.size()] << 8)))
        {
            rxReadIndex = (frameStart + 1) % ProcBuffer.size();
            continue;
        }

        size_t copyLen = (payloadSize < maxLen) ? payloadSize : maxLen;
        size_t payloadStartIndex = (lenIdx + 1) % ProcBuffer.size();
        for (size_t i = 0; i < copyLen; ++i) {
            pOut[i] = ProcBuffer[(payloadStartIndex + i) % ProcBuffer.size()];
        }

        rxReadIndex = (frameStart + totalFrameLength) % ProcBuffer.size();
        lastActivityTick = HAL_GetTick();

        return payloadSize;
    }
    return 0;
}

/**
 * @brief  [接收包装器] 从协议栈提取数据并存入 std::vector
 * * @details 这是一个方便上层业务调用的包装函数。它利用栈上的临时缓冲区作为中转，
 * 调用核心引擎 @ref ReceiveRaw 进行解析。
 * * @param  outPayload [输出] 用于存放解析后有效载荷的 vector 容器。
 * 若解析成功，容器内容将被重写。
 * * @return true       成功提取到一帧完整数据。
 * @return false      当前无完整帧或解析失败。
 * * @note
 * - **内存警告**：由于使用了 std::vector，此函数会涉及堆内存（Heap）的申请与释放，
 * 不建议在极高频率（如 1ms 级的控制环路）中调用。
 * - **长度限制**：受限于内部静态栈空间，此包装器最大支持解析 256 字节的 Payload。
 * 若需处理更大报文，请直接调用 @ref ReceiveRaw。
 * - 此函数是线程不安全的，建议在固定的通讯处理任务中调用。
 */
bool ProtocolBase::Receive(std::vector<uint8_t>& outPayload)
{
    uint8_t temp[256]; // 临时栈空间
    size_t len = ReceiveRaw(temp, 256);
    if (len > 0) {
        outPayload.assign(temp, temp + len);
        return true;
    }
    return false;
}

/**
 * @brief  处理具体的通讯异常
 * @param  err: 错误类型枚举
 */
void ProtocolBase::HandleHardwareError(CommError err)
{
    errorCount++;
    lastError = err;
    switch (err) {
    case CommError::BUFFER_OVERFLOW:
        rxReadIndex = rxWriteIndex;
        break;

    case CommError::HARDWARE_FAILURE:
        break;
    default:
        break;
    }
    OnCommFailure(err);
}

/**
 * @brief  发送字符串包装函数
 * @param  str: 要发送的 std::string
 * @note   将字符串转换为 vector<uint8_t> 后调用通用发送函数
 */
void ProtocolBase::SendString(const std::string& str)
{
    SendWithCRC16(std::vector<uint8_t>(str.begin(), str.end()));
}

/**
 * @brief  发送数值（自动转字符串）
 * @param  number   : 要发送的数值
 * @param  precision: 小数点后保留几位 (0 表示整数)
 * @note   例如 SendNumber(3.1415, 2) -> 发送字符串 "3.14"
 */
void ProtocolBase::SendNumber(float number, int precision)
{
    char buf[32];
    if (precision == 0) sprintf(buf, "%d", (int)number);
    else sprintf(buf, "%.*f", precision, number);
    SendString(std::string(buf));
}

/**
 * @brief 发送标准应答包 (ACK)
 * @param targetCmdId  上位机发过来的那条指令的 ID (告诉它我回的是哪句话)
 * @param status       执行结果
 */
void ProtocolBase::SendACK(uint8_t targetCmdId, AckStatus status)
{
    // 准备载荷：3 个字节
    std::array<uint8_t, 3> payload{};

    payload[0] = CMD_ACK_ID;            // 这是一条 ACK 包
    payload[1] = targetCmdId;           // 针对的是哪条指令
    payload[2] = static_cast<uint8_t>(status); // 结果是什么

    // 直接调用万能发送接口，自动计算 CRC 并发出
    SendArrayWithCRC16(payload);
}

/**
 * @brief  通用发送函数：打包数据并发送
 * @param  data: 原始有效载荷 (不含头尾和校验)
 *
 * 打包过程：
 * 1. 添加帧头 (0xAA 0x55)
 * 2. 添加长度字节 (Payload 长度)
 * 3. 填充 Payload
 * 4. 计算 [长度+Payload] 的 CRC16
 * 5. 添加 CRC (低字节在前)
 * 6. 添加帧尾 (0x55 0xAA)
 * 7. 调用虚函数 RawSend() 发送最终字节流
 */
void ProtocolBase::SendWithCRC16(const std::vector<uint8_t>& data)
{
    if (data.empty()) return;
    if (data.size() > 250) return;

    std::vector<uint8_t> arr;
    arr.reserve(10 + data.size());


    arr.push_back(FRAME_HEADER1);
    arr.push_back(FRAME_HEADER2);

    arr.push_back(static_cast<uint8_t>(data.size()));

    arr.insert(arr.end(), data.begin(), data.end());

    uint16_t crc = CRC16_Modbus(arr.data() + 2, data.size() + 1);

    arr.push_back(crc & 0xFF);
    arr.push_back((crc >> 8) & 0xFF);

    arr.push_back(FRAME_TAIL1);
    arr.push_back(FRAME_TAIL2);

    SafeRawSend(arr.data(), arr.size());
}
