/**
  ******************************************************************************
  * @file    Flash.cpp
  * @brief   通用 Flash 驱动实现层
  * @note    支持 STM32F1/F4/G0/L4/H7 等多系列芯片的自动擦写管理
  ******************************************************************************
  */

#include "FlashDriver.h"
#include <cstdio>


static uint32_t primask_bit;
static void EnterCriticalSection() {
    primask_bit = __get_PRIMASK();
    __disable_irq();
}
static void ExitCriticalSection() {
    __set_PRIMASK(primask_bit);
}

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
/**
 * @brief  [内部函数] 将物理地址映射为扇区编号
 * @param  Address Flash 物理地址
 * @return 扇区索引 (如 FLASH_SECTOR_0)
 */
static uint32_t GetSector(uint32_t Address);
#endif



/**
 * @brief  向 Flash 写入原始数据缓冲区
 * @param  addr          写入起始物理地址 (必须 4 字节对齐)
 * @param  data          源数据指针 (32位指针)
 * @param  lengthInWords 要写入的字(Word)数量
 * @return true: 成功; false: 擦除或写入失败
 * * @note   【核心逻辑】
 * 1. 自动计算数据跨越了多少页(Page)或扇区(Sector)。
 * 2. 执行擦除操作 (Erase)。
 * 3. 执行编程操作 (Program)。
 * 4. 全程使用 RAII 锁保护，防止异常退出导致 Flash 未上锁。
 */
bool FlashDriver::WriteBuffer(uint32_t addr, const uint32_t* data, size_t lengthInWords, bool verify) {
    if (lengthInWords == 0) return true;

    ScopedLock lock;

    EnterCriticalSection();

#if defined(STM32F1)
    // F1 系列: 只有 PGERR (编程错误) 和 WRPERR (写保护错误)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

#elif defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    // 高性能系列: 错误标志更丰富
    // OPERR: 操作错误
    // WRPERR: 写保护错误
    // PGAERR: 对齐错误 (Alignment)
    // PGPERR: 并行位数错误 (Parallelism)
    // PGSERR: 顺序错误 (Sequence)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

#elif defined(STM32L4) || defined(STM32G0) || defined(STM32G4) || defined(STM32WB)
    // 新低功耗系列: 通常提供了 ALL_ERRORS 宏
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

#else
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
#endif

    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;
    bool eraseSuccess = false;

    uint32_t endAddr = addr + lengthInWords * 4 - 1;

#if defined(STM32F1) || defined(STM32G0) || defined(STM32L4)
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 0x800
#endif
    uint32_t startPage = (addr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    uint32_t endPage   = (endAddr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    uint32_t nbPages   = (endPage - startPage) / FLASH_PAGE_SIZE + 1;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = startPage;
    EraseInitStruct.NbPages     = nbPages;
#elif defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    uint32_t startSector = GetSector(addr);
    uint32_t endSector   = GetSector(endAddr);
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector       = startSector;
    EraseInitStruct.NbSectors    = endSector - startSector + 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#endif

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        eraseSuccess = true;
    }
    if (!eraseSuccess) {
        ExitCriticalSection();
        return false;
    }


    for (size_t i = 0; i < lengthInWords; i++) {
        uint32_t currentAddr = addr + i * 4;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddr, data[i]) != HAL_OK) {
            ExitCriticalSection();
            return false;
        }
    }

    ExitCriticalSection();

#if defined(STM32F7) || defined(STM32H7)
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr, lengthInWords * 4);
#endif

    if (verify) {
        for (size_t i = 0; i < lengthInWords; i++) {
            uint32_t flashVal = *(__IO uint32_t*)(addr + i * 4);
            if (flashVal != data[i]) {
                return false;
            }
        }
    }

    return true;
}

// =============================================================================
//                             架构相关辅助函数
// =============================================================================

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
/**
 * @brief  将 Flash 物理地址映射为扇区索引 (Sector Index)
 * @param  Address 物理地址 (如 0x08008000)
 * @return 扇区宏 (如 FLASH_SECTOR_1)
 * * @warning 【移植警告】
 * 此映射表基于 STM32F405/407/415/417 (1MB Flash, Single Bank)。
 * 如果使用 STM32F429 (Dual Bank) 或 STM32F401/411，扇区划分可能不同。
 * 请务必查阅 Reference Manual 的 "Flash module organization" 章节进行核对。
 */
static uint32_t GetSector(uint32_t Address) {
    uint32_t sector = 0;

    // F4 Standard Sector Map (Single Bank Mode)
    // Sector 0-3: 16KB
    // Sector 4:   64KB
    // Sector 5-11: 128KB
    if(Address < 0x08004000)      sector = FLASH_SECTOR_0;
    else if(Address < 0x08008000) sector = FLASH_SECTOR_1;
    else if(Address < 0x0800C000) sector = FLASH_SECTOR_2;
    else if(Address < 0x08010000) sector = FLASH_SECTOR_3;
    else if(Address < 0x08020000) sector = FLASH_SECTOR_4;
    else if(Address < 0x08040000) sector = FLASH_SECTOR_5;
    else if(Address < 0x08060000) sector = FLASH_SECTOR_6;
    else if(Address < 0x08080000) sector = FLASH_SECTOR_7;
    else if(Address < 0x080A0000) sector = FLASH_SECTOR_8;
    else if(Address < 0x080C0000) sector = FLASH_SECTOR_9;
    else if(Address < 0x080E0000) sector = FLASH_SECTOR_10;
    else                          sector = FLASH_SECTOR_11;

    return sector;
}
#endif