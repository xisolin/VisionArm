/**
  ******************************************************************************
  * @file    Flash.h
  * @brief   STM32 内部 Flash 读写驱动封装 (C++11 RAII & Template)
  * @version V2.0
  * @date    2026-01-16
  * @note    
  * 1. 包含了类型安全检查 (static_assert)，防止写入非 POD 类型。
  * 2. 使用 RAII 机制管理 Flash 锁，防止异常退出导致 Flash 处于未锁状态。
  * 3. 底层依赖 STM32 HAL 库。
  ******************************************************************************
  */

#ifndef __FLASH_H
#define __FLASH_H

#pragma once

#include "main.h"
#include <type_traits>
#include <vector>
#include <cstring>

/**
 * @class FlashDriver
 * @brief Flash 操作静态工具类
 */
class FlashDriver {
public:
    // =========================================================================
    //                            RAII 锁机制
    // =========================================================================

    /**
     * @class ScopedLock
     * @brief Flash 解锁/上锁的作用域守卫 (Resource Acquisition Is Initialization)
     * @note  在构造时自动解锁 Flash，在析构（作用域结束）时自动上锁。
     * 即使函数中有多个 return 语句，也能保证 Flash 最终被安全锁定。
     */
    class ScopedLock {
    public:
        /**
         * @brief 构造函数：立即解锁 Flash
         */
        ScopedLock() { 
            HAL_FLASH_Unlock(); 
        }

        /**
         * @brief 析构函数：作用域结束时自动锁定 Flash
         */
        ~ScopedLock() { 
            HAL_FLASH_Lock(); 
        }

        // 禁止拷贝构造和赋值操作，防止锁状态管理混乱
        ScopedLock(const ScopedLock&) = delete;
        ScopedLock& operator=(const ScopedLock&) = delete;
    };

    // =========================================================================
    //                            模板方法 (Type-Safe APIs)
    // =========================================================================

    /**
     * @brief  向 Flash 写入任意类型的对象
     * @tparam T 数据类型 (必须是 Trivially Copyable / POD 类型)
     * @param  addr Flash 写入的起始物理地址 (建议 4 字节对齐)
     * @param  data 要写入的数据引用
     * @return true: 写入成功; false: 写入失败
     * * @note   【编译期检查】
     * 使用 static_assert 检查类型 T 是否可以直接内存拷贝。
     * - 允许: int, float, double, struct { int x; }, int arr[10]
     * - 禁止: std::string, std::vector, 带有 virtual 函数的类
     * * @example
     * struct Config { int baud; float pid_p; };
     * Config cfg = {115200, 1.5f};
     * FlashDriver::Write(0x08010000, cfg);
     */
    template <typename T>
    static bool Write(uint32_t addr, const T& data) {
        static_assert(std::is_trivially_copyable<T>::value, 
                      "Error: Data type is not trivially copyable (contains pointers, vtable, or non-trivial constructors).");

        size_t wordCount = (sizeof(T) + 3) / 4;

        return WriteBuffer(addr, reinterpret_cast<const uint32_t*>(&data), wordCount);
    }

    /**
     * @brief  从 Flash 读取任意类型的对象
     * @tparam T 数据类型
     * @param  addr Flash 读取的起始物理地址
     * @param  data [OUT] 接收数据的对象引用
     * * @note   使用 std::memcpy 进行拷贝，相比直接指针强转 (*(T*)addr) 更安全，
     * 避免了 strict-aliasing 违规风险，且编译器会优化为高效的加载指令 (LDR)。
     */
    template <typename T>
    static void Read(uint32_t addr, T& data) {
        static_assert(std::is_trivially_copyable<T>::value, "Error: Data type is not trivially copyable.");

        std::memcpy(&data, reinterpret_cast<const void*>(addr), sizeof(T));
    }

    // =========================================================================
    //                            底层接口 (Low-Level API)
    // =========================================================================

    /**
     * @brief  写入原始数据缓冲区 (底层实现)
     * @param  addr          Flash 起始地址 (必须按照 FLASH_TYPEPROGRAM_WORD 对齐)
     * @param  data          数据源指针 (32位数组)
     * @param  lengthInWords 要写入的 32位字 (Word) 数量
     * @return true: 全部写入成功; false: 擦除或编程失败
     * * @warning 
     * 1. 此函数会自动擦除目标区域 (具体策略需在 .cpp 中根据芯片型号实现: 页擦除 vs 扇区擦除)。
     * 2. 写入期间 CPU 会暂停运行 (Stall)，请勿在对实时性要求极高的中断中调用。
     */
    static bool WriteBuffer(uint32_t addr, const uint32_t* data, size_t lengthInWords, bool verify = true);
};

#endif // __FLASH_H