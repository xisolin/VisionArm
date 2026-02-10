#ifndef SIMPLESHELL_H
#define SIMPLESHELL_H

#pragma once

#include <vector>
#include <string>
#include <cstring>
#include <functional>
#include <cstdarg>
#include <cstdio>

// 定义配置
#define SHELL_MAX_LINE_LEN  80   // 命令行最大长度
#define SHELL_MAX_ARGS      10   // 最大参数个数

class SimpleShell {
public:
    // 定义命令函数的签名：int func(int argc, char* argv[])
    using CommandFunc = std::function<int(int, char**)>;

    struct CommandEntry {
        const char* name;
        const char* help;
        CommandFunc handler;
    };

    SimpleShell();
    virtual ~SimpleShell() = default;

    void Register(const char* name, CommandFunc handler, const char* help = "");
    void Input(char ch);
    void Printf(const char* fmt, ...);

protected:
    virtual void Write(const char* data, int len) = 0;

private:
    char _rxBuffer[SHELL_MAX_LINE_LEN];
    int  _rxIndex;
    std::vector<CommandEntry> _commands;

    // 内部处理函数
    void ProcessLine();
    void PrintPrompt();
};

#endif // SIMPLESHELL_H