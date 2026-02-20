#ifndef SIMPLESHELL_H
#define SIMPLESHELL_H

#pragma once

#include "main.h"
#include <vector>
#include <string>
#include <cstring>
#include <functional>
#include <cstdarg>
#include <cstdio>
#include "CrashLog.h"

// ================= 配置区 =================
#define SHELL_MAX_LINE_LEN  128   // 命令行最大长度
#define SHELL_MAX_ARGS      12   // 最大参数个数

// =================  定义通用硬件接口 =================
struct IShellPort {
    virtual void Write(const char* data, int len) = 0;

    void PrintRaw(const char* fmt, ...) {
        char buf[64];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (len > 0) Write(buf, len);
    }
    virtual ~IShellPort() = default;
};

// ================= 2. Shell 核心逻辑类 (大脑) =================
class SimpleShell {
public:
    // 定义命令函数的签名
    using CommandFunc = std::function<int(int, char**)>;

    struct CommandEntry {
        const char* name;
        const char* help;
        CommandFunc handler;
    };

    /**
     * @brief 构造函数
     * @param port 默认的输出端口 (可选，可以先传 nullptr)
     */
    explicit SimpleShell(IShellPort* port = nullptr);
    virtual ~SimpleShell() = default;

    void Init();
    void Input(char ch);
    void Printf(const char* fmt, ...);
    void Register(const char* name, const CommandFunc& handler, const char* help = "");

    /**
     * @brief [核心新功能] 动态切换输出端口
     * @details 比如：收到网口数据时切到网口，收到串口数据时切到串口
     */
    void SetPort(IShellPort* port) { _port = port; }

private:
    IShellPort* _port; // 当前激活的输出端口 (指针)

    char _rxBuffer[SHELL_MAX_LINE_LEN]{};
    int  _rxIndex;
    std::vector<CommandEntry> _commands;

    // 内部处理函数
    void ProcessLine();
    void PrintPrompt();
    void Write(const char* data, int len);
};

#endif // SIMPLESHELL_H