#include "MT6816_base.h"

// 初始化 MT6816 编码器
bool MT6816Base:: Init()
{
    // 1. 初始化 SPI 接口
    SpiInit();
    // 2. 读取一次角度数据，确保硬件能正常通信
    UpdateAngle();

    // 3. 检查快速校准表数据是否有效
    // Check if the stored calibration data are valid
    angleData.rectifyValid = true;// 默认认为校准有效
    for (uint32_t i = 0; i < RESOLUTION; i++)
    {
        // 如果某个校准值为 0xFFFF，说明校准无效
        if (quickCaliDataPtr[i] == 0xFFFF)
            angleData.rectifyValid = false;
    }

    // 返回校准是否有效
    return angleData.rectifyValid;
}

// 更新编码器角度数据
uint16_t MT6816Base::UpdateAngle()
{
    // 1. 构造 SPI 发送数据，读取角度寄存器
    dataTx[0] = (0x80 | 0x03) << 8;// 读取寄存器 3
    dataTx[1] = (0x80 | 0x04) << 8;// 读取寄存器 4
    // 2. 尝试读取三次 SPI 数据，直到校验成功
    for (uint8_t i = 0; i < 3; i++)
    {
        // 发送 SPI 并读取 16 位数据
        dataRx[0] = SpiTransmitAndRead16Bits(dataTx[0]);
        dataRx[1] = SpiTransmitAndRead16Bits(dataTx[1]);
        // 将两次读取的数据拼接成 16 位原始数据
        spiRawData.rawData = ((dataRx[0] & 0x00FF) << 8) | (dataRx[1] & 0x00FF);

        //奇偶校验
        hCount = 0;
        for (uint8_t j = 0; j < 16; j++)
        {
            if (spiRawData.rawData & (0x0001 << j))
                hCount++;
        }
        // 如果奇偶校验失败，标记 checksumFlag = false
        if (hCount & 0x01)
        {
            spiRawData.checksumFlag = false;
        } else
        {
            // 校验成功，标记 checksumFlag = true 并退出循环
            spiRawData.checksumFlag = true;
            break;
        }
    }
    // 4. 如果校验成功，解析原始角度和无磁标志
    if (spiRawData.checksumFlag)
    {
        // 原始角度值为 rawData 右移 2 位（14 位有效）
        spiRawData.rawAngle = spiRawData.rawData >> 2;
        // 无磁信号标志位在 bit1
        spiRawData.noMagFlag = (bool) (spiRawData.rawData & (0x0001 << 1));
    }
    // 5. 更新基类成员 angleData
    angleData.rawAngle = spiRawData.rawAngle;// 原始角度
    angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];// 校正后的角度
    // 6. 返回校正后的角度值
    return angleData.rectifiedAngle;
}

// 判断编码器是否已完成校准
bool MT6816Base::IsCalibrated()
{
    return angleData.rectifyValid;
}



