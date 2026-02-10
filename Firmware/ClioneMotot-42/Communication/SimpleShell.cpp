#include "SimpleShell.h"

SimpleShell::SimpleShell() : _rxIndex(0) {
    // 可以在这里注册内置命令，比如 help
    Register("help", [this](int argc, char** argv) -> int {
        Printf("Available commands:\r\n");
        for (const auto& cmd : _commands) {
            Printf("  %-10s : %s\r\n", cmd.name, cmd.help);
        }
        return 0;
    }, "List all commands");
}

void SimpleShell::Register(const char* name, CommandFunc handler, const char* help) {
    _commands.push_back({name, handler, help});
}

void SimpleShell::Input(char ch) {
    // 1. 处理回车换行 (命令结束)
    if (ch == '\r' || ch == '\n') {
        if (_rxIndex > 0) {
            Printf("\r\n"); // 换行
            _rxBuffer[_rxIndex] = '\0'; // 字符串结韦
            ProcessLine(); // 解析并执行
            _rxIndex = 0;  // 清空缓冲区
        } else {
            Printf("\r\n");
        }
        PrintPrompt();
    }
    // 2. 处理退格键 (Backspace / Delete)
    else if (ch == '\b' || ch == 0x7F) {
        if (_rxIndex > 0) {
            _rxIndex--;
            // 终端技巧：退格、打印空格、再退格，实现视觉删除
            Write("\b \b", 3); 
        }
    }
    // 3. 普通字符 (存入缓冲区)
    else if (_rxIndex < SHELL_MAX_LINE_LEN - 1) {
        // 过滤不可打印字符
        if (ch >= 32 && ch <= 126) {
            _rxBuffer[_rxIndex++] = ch;
            Write(&ch, 1); // 回显 (Echo)
        }
    }
}

void SimpleShell::ProcessLine() {
    char* argv[SHELL_MAX_ARGS];
    int argc = 0;

    // 使用 strtok 切割字符串 (注意：strtok 会修改原字符串)
    char* token = std::strtok(_rxBuffer, " ");
    while (token != nullptr && argc < SHELL_MAX_ARGS) {
        argv[argc++] = token;
        token = std::strtok(nullptr, " ");
    }

    if (argc == 0) return;

    // 查找命令
    for (const auto& cmd : _commands) {
        if (std::strcmp(argv[0], cmd.name) == 0) {
            // 执行回调
            int ret = cmd.handler(argc, argv);
            if (ret != 0) {
                Printf("Error: Command returned %d\r\n", ret);
            }
            return;
        }
    }

    Printf("Unknown command: '%s'. Type 'help' for list.\r\n", argv[0]);
}

void SimpleShell::Printf(const char* fmt, ...) {
    char buf[128]; // 临时缓冲区
    va_list args;
    va_start(args, fmt);
    int len = std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        Write(buf, len);
    }
}

void SimpleShell::PrintPrompt() {
    Write(">> ", 3);
}