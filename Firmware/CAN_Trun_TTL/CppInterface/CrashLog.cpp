/**
  ******************************************************************************
  * @file    CrashLog.cpp
  * @brief   系统崩溃“黑匣子”记录服务 (Black Box Recording Service)
  * @note
  * 1. 提供 C 语言接口 (extern "C") 供 main.c 或 HAL 回调函数调用。
  * 2. 利用 FlashDriver 实现崩溃现场信息的持久化存储。
  * 3. 包含自动重启机制 (System Reset)。
  ******************************************************************************
  */

#include "CrashLog.h"
#include "FlashDriver.h"
#include "main.h"
#include <cstring>
#include <cstdio>

/**
  * @brief  [核心功能] 记录崩溃现场并复位系统
  * @param  file  发生错误的文件名 (__FILE__)
  * @param  line  发生错误的行号 (__LINE__)
  * @return None  (此函数执行后系统将复位，无返回值)
  * * @note   【执行流程】
  * 1. 填充魔数 (Magic Code) 和时间戳，标记这是一条有效日志。
  * 2. 截断拷贝文件名，防止缓冲区溢出。
  * 3. 调用 FlashDriver 将数据原子写入 Flash 指定区域。
  * 4. 执行软复位 (NVIC_SystemReset)，尝试恢复系统运行。
  * * @warning 【Flash 寿命警告】
  * 只有在致命错误 (Fatal Error) 时才调用此函数。
  * 如果系统陷入“上电即崩溃”的死循环，Flash 可能会在短时间内耗尽擦写寿命 (通常 10万次)。
  * 建议在应用层增加“连续崩溃保护”逻辑（如检测到连续 Crash 3次则停机）。
  */
void Log_RecordCrash(const char* file, uint32_t line) {
    CrashInfo_t log;

    log.magic = CRASH_MAGIC;
    log.timestamp = HAL_GetTick();
    log.line = line;

    const char* fileName = file;

    const char* slash1 = strrchr(file, '/');
    const char* slash2 = strrchr(file, '\\');

    const char* lastSlash = (slash1 > slash2) ? slash1 : slash2;

    if (lastSlash != NULL) {
        fileName = lastSlash + 1; // 指针后移一位，跳过斜杠
    }
    // =======================================================

    memset(log.file, 0, sizeof(log.file));

    strncpy(log.file, fileName, sizeof(log.file) - 1);

    FlashDriver::Write(CRASH_LOG_ADDR, log);
    NVIC_SystemReset();
}

/**
  * @brief  [上电调用] 检查并打印上一次的崩溃日志
  * @param  None
  * @retval None
  * * @note   通常在 main() 函数的 HAL_Init() 之后，while(1) 之前调用。
  * 如果检测到有效的 Magic Number，说明上次是异常复位，将通过 printf 输出日志。
  * 输出完成后，会清除 Flash 中的标志位，避免下次正常启动时重复误报。
  */
void Log_CheckAndPrint(void) {
    CrashInfo_t log;

    FlashDriver::Read(CRASH_LOG_ADDR, log);

    if (log.magic == CRASH_MAGIC) {
        // --- 发现崩溃记录，打印诊断信息 ---
        printf("\r\n========================================\r\n");
        printf("[CRITICAL] System Recovered from Crash!\r\n"); // 严重级别提示
        printf("----------------------------------------\r\n");
        printf("Source File : %s\r\n", log.file);
        printf("Error Line  : %lu\r\n", log.line);
        printf("Uptime      : %lu ms\r\n", log.timestamp);
        printf("========================================\r\n");

        log.magic = 0;
        FlashDriver::Write(CRASH_LOG_ADDR, log);

    } else {
        printf("[INFO] System Normal Boot.\r\n");
    }
}

/**
  * @brief  [系统级] 致命错误统一处理入口 (Fatal Error Handler)
  * @param  file  产生错误的文件名 (通常由 __FILE__ 传入)
  * @param  line  产生错误的行号 (通常由 __LINE__ 传入)
  * @return None  (此函数不会返回，最终将触发系统复位)
  * @note
  * 1. 此函数是不可重入的 (Non-reentrant)，一旦进入将接管系统控制权。
  * 2. 这里的 printf 必须是轮询(阻塞)模式，因为中断已被关闭。
  * 3. 包含了完整的"关中断 -> 记录 -> 复位"流程，确保现场数据不被破坏。
  */
void Log_FatalError(const char* file, uint32_t line) {
    //关闭全局中断
    __disable_irq();

    // 打印“遗言” (阻塞式打印)
    printf("\r\n[FATAL] System Crash at %s:%lu\r\n", file, line);
    printf("Saving log and resetting...\r\n");

    // 记录到 Flash 并复位
    Log_RecordCrash(file, line);

    while(1) {}
}

/**
  * @brief  擦除 Flash 中的崩溃日志区域
  * @note   此函数执行以下操作：
  * 1. 解锁 Flash 控制寄存器
  * 2. 根据芯片型号自动选择 "页擦除(Page)" 或 "扇区擦除(Sector)"
  * 3. 擦除指定区域并重新上锁
  * @note   【警告】Flash 擦除是阻塞式操作，且耗时较长（几十毫秒到几百毫秒）。
  * 在此期间，如果代码运行在 Flash 中，CPU 将暂停取指，中断响应可能会延迟。
  * @retval None
  */
void Log_Clear(void)
{
    // 1. 解锁 Flash
    HAL_FLASH_Unlock();

    // 2. 初始化擦除结构体
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;

    // ============================================
    // 分支 A: STM32F4 系列 (按扇区擦除)
    // ============================================
#if defined(CHIP_MODEL_F407_1M) || defined(CHIP_MODEL_F407_512K)
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector       = CRASH_LOG_SECTOR; // 必须用扇区号
    EraseInitStruct.NbSectors    = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 通常是 2.7V~3.6V

    // ============================================
    // 分支 B: STM32F1 系列 (按页擦除)
    // ============================================
#else
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = CRASH_LOG_ADDR;    // F1用地址
    EraseInitStruct.NbPages     = 1;
#endif

    // 3. 执行擦除 (阻塞式)
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        // 如果擦除失败，可以在这里处理，比如点亮红色LED
        // Error_Handler();
    }

    // 4. 重新上锁
    HAL_FLASH_Lock();
}