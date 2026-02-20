#ifndef CRASH_LOG_H
#define CRASH_LOG_H

#pragma once

#include <stdint.h>

// 魔数，用于判断 Flash 里存的是不是有效日志
#define CRASH_MAGIC  0xDEADBEEF

/* 1. STM32F407 系列 (Flash = 1MB / 512KB) */
// #define CHIP_MODEL_F407_1M    // 适用于 F407ZG, F407VG (1MB Flash)
// #define CHIP_MODEL_F407_512K  // 适用于 F407VE, F407ZE (512KB Flash)

/* 2. STM32F103 大容量 (High Density) */
// #define CHIP_MODEL_F103RC     // 适用于 F103RCT6 (256KB Flash, Page=2KB)
// #define CHIP_MODEL_F103RE     // 适用于 F103RET6 (512KB Flash, Page=2KB)

/* 3. STM32F103 中容量 (Medium Density) */
#define CHIP_MODEL_F103CB     // 适用于 F103CBT6 (128KB Flash, Page=1KB)
// #define CHIP_MODEL_F103C8     // 适用于 F103C8T6 (64KB Flash, Page=1KB)



#if defined(CHIP_MODEL_F407_1M)
    // F407 (1MB): 最后一个扇区是 Sector 11 (128KB)
    // 起始地址: 0x080E0000
    #define CRASH_LOG_ADDR  0x080E0000
    #define CRASH_LOG_SECTOR  FLASH_SECTOR_11
#elif defined(CHIP_MODEL_F407_512K)
    // F407 (512KB): 最后一个扇区是 Sector 7 (128KB)
    // 起始地址: 0x08060000
    #define CRASH_LOG_ADDR  0x08060000
    #define CRASH_LOG_SECTOR  FLASH_SECTOR_7

#elif defined(CHIP_MODEL_F103RC)
    // F103RC (256KB): 最后一页起始地址 = 0x08000000 + 256K - 2K
    // 0x08040000 - 0x800 = 0x0803F800
    #define CRASH_LOG_ADDR  0x0803F800

#elif defined(CHIP_MODEL_F103RE)
    // F103RE (512KB): 最后一页起始地址
    // 0x08080000 - 0x800 = 0x0807F800
    #define CRASH_LOG_ADDR  0x0807F800

#elif defined(CHIP_MODEL_F103CB)
    // F103CB (128KB): 最后一页起始地址 (Page=1KB)
    // 0x08020000 - 0x400 = 0x0801FC00
    #define CRASH_LOG_ADDR  0x0801FC00

#elif defined(CHIP_MODEL_F103C8)
    // F103C8 (64KB): 最后一页起始地址 (Page=1KB)
    // 0x08010000 - 0x400 = 0x0800FC00
    #define CRASH_LOG_ADDR  0x0800FC00

#else
    #error "Please select a CHIP_MODEL in CrashLog.h!"
#endif

typedef struct {
    uint32_t magic;         // 标志位
    uint32_t timestamp;     // 崩溃发生的时间 (HAL_GetTick)
    uint32_t line;          // 行号
    char     file[64];      // 文件名 (截断保存，防止溢出)
} CrashInfo_t;

#ifdef __cplusplus
extern "C" {
#endif

    /**
 * @brief  [终极接口] 发生致命错误时调用
 * @note   此函数会执行：关中断 -> 打印日志 -> 写Flash -> 系统复位
 */
    void Log_FatalError(const char* file, uint32_t line);

    /**
     * @brief  [辅助宏] 自动填充当前文件名和行号
     * @usage  直接在代码里调用 LOG_CRASH_HERE(); 即可
     */
#define LOG_CRASH_HERE()  Log_FatalError(__FILE__, __LINE__)

    // [C 接口] 记录崩溃并重启
    void Log_RecordCrash(const char* file, uint32_t line);

    // [C 接口] 检查是否有崩溃记录 (上电调用)
    void Log_CheckAndPrint(void);

    void Log_Clear(void);

#ifdef __cplusplus
}
#endif

#endif // CRASH_LOG_H