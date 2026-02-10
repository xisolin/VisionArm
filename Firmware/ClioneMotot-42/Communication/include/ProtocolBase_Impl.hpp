#ifndef PROTOCOLBASE_IMPL_HPP
#define PROTOCOLBASE_IMPL_HPP

#include "ProtocolBase.h"

/**
 * @brief [通用解析器] 将接收到的原始数据转换为指定结构体
 * @tparam T      目标结构体类型 (如 Packet_MovePos)
 * @param buffer  原始数据指针
 * @param len     数据长度 (用于安全检查)
 * @return        转换后的结构体副本
 */
template <typename T>
T ProtocolBase::ParseStruct(const uint8_t* buffer, size_t len)
{
    if (len < sizeof(T)) {
        T empty{};
        return empty;
    }
    T result;
    memcpy(&result, buffer, sizeof(T));
    return result;
}

/**
 * @brief 发送数组模板函数 (自动计算 CRC 并打包)
 * @tparam T 数据类型 (如 uint8_t, float, 自定义结构体)
 * @tparam N 数组大小
 * @param arr std::array 数组
 */
template <typename T, size_t N>
void ProtocolBase::SendArrayWithCRC16(const std::array<T, N>& arr)
{
    static_assert(std::is_standard_layout<T>::value && std::is_trivially_copyable<T>::value,
                  "Error: Only POD types allowed to be sent over-the-wire");

    constexpr size_t DataLen = N * sizeof(T);
    constexpr size_t FrameLen = 2 + 1 + DataLen + 2 + 2;

    uint8_t buffer[FrameLen];

    size_t idx = 0;
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = static_cast<uint8_t>(DataLen);

    const auto* ptr = reinterpret_cast<const uint8_t*>(arr.data());
    memcpy(&buffer[idx], ptr, DataLen);
    idx += DataLen;

    uint16_t crc = CRC16_Modbus(&buffer[2], DataLen + 1);

    buffer[idx++] = crc & 0xFF;
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = FRAME_TAIL1;
    buffer[idx++] = FRAME_TAIL2;

    RawSend(buffer, FrameLen);
}


/**
 * @brief  [接收包装器] 从协议栈提取数据并存入定长数组 (零堆内存分配)
 * * @details 这是一个高性能、内存安全的包装函数，专门为下位机实时任务设计。
 * 它直接利用外部提供的 std::array 空间进行解析，完全规避了动态内存申请（malloc/new）。
 * * @tparam N            [模板参数] 目标数组的容量，由编译器自动推导。
 * @param  outArray     [输出] 用于存放有效载荷的 std::array 引用。
 * * @return true         成功提取到一帧完整数据，且数据长度有效。
 * @return false        当前无完整帧、校验失败或正在等待数据补齐。
 * * @note
 * - **高性能**：此函数是核心引擎 @ref ReceiveRaw 的直接封装，没有任何中间拷贝开销。
 * - **确定性**：执行时间不依赖于堆状态，适用于 1ms 级的运动控制循环。
 * - **安全性**：模板参数 N 自动充当了边界卫兵，确保解析出的 Payload 不会溢出外部数组。
 * - 只有在完整解析并校验通过后，读指针才会被移动。
 */
template <size_t N>
bool ProtocolBase::Receive(std::array<uint8_t, N>& outArray)
{
    return (this->ReceiveRaw(outArray.data(), N) > 0);
}


#endif // PROTOCOLBASE_IMPL_HPP
