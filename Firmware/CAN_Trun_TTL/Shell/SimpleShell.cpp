#include "SimpleShell.h"
#include <cstring>

// 构造函数：初始化端口和内置命令
SimpleShell::SimpleShell(IShellPort* port) : _port(port), _rxIndex(0)
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
}

void SimpleShell::Init()
{
    Register("help", [this](int argc, char** argv) -> int
    {
        Printf("Available commands:\r\n");
        for (const auto& cmd : _commands)
        {
            Printf("  %-10s : %s\r\n", cmd.name, cmd.help);
        }
        return 0;
    }, "List all commands");

    Register("clear", [this](int argc, char** argv) -> int
    {
        Write("\033[2J\033[H", 7);
        return 0;
    }, "Clear screen");

    Register("log", [this](int argc, char** argv) -> int
    {
        volatile auto* pLog = (volatile CrashInfo_t*)CRASH_LOG_ADDR;

        if (pLog->magic != CRASH_MAGIC)
        {
            Printf("[System Normal] No crash record found.\r\n");
            return 0;
        }

        Printf("\r\n");
        Printf("=== SYSTEM CRASH LOG ===\r\n");
        Printf("Time : %d.%03d s\r\n", pLog->timestamp / 1000, pLog->timestamp % 1000);
        Printf("File : %.*s\r\n", sizeof(pLog->file), pLog->file);
        Printf("Line : %d\r\n", pLog->line);
        Printf("========================\r\n");

        return 0;
    }, "Show crash log");


    Register("log_clr", [this](int argc, char** argv) -> int
    {
        Log_Clear();
        Printf("To clear log, please implement Log_Clear() in CrashLog.c\r\n");
        return 0;
    }, "Clear crash log");


    Printf("\r\n");
    Printf("============================\r\n");
    Printf("   System Shell Ready\r\n");
    Printf("============================\r\n");

    // D. 打印提示符
    PrintPrompt();
}

void SimpleShell::Register(const char* name, const CommandFunc& handler, const char* help)
{
    _commands.push_back({name, help, handler});
}

void SimpleShell::Write(const char* data, int len)
{
    if (_port)
    {
        _port->Write(data, len);
    }
}

void SimpleShell::Input(char ch)
{
    if (ch == '\n') return;
    switch (ch)
    {
    case '\r':
        if (_rxIndex > 0)
        {
            _rxBuffer[_rxIndex] = '\0';
            ProcessLine();
            _rxIndex = 0;
        }
        PrintPrompt();
        break;
    case '\b':
    case 0x7F:
        if (_rxIndex > 0)
        {
            _rxIndex--;
            Write("\b \b", 3);
        }
        break;
    default:
        if (ch >= ' ' && ch <= '~')
        {
            if (_rxIndex < (SHELL_MAX_LINE_LEN - 1))
            {
                _rxBuffer[_rxIndex++] = ch;
            }
            else
            {
                Write("Exceeding the instruction length", sizeof ("Exceeding the instruction length"));
            }
        }
        break;
    }
}

void SimpleShell::Printf(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) Write(buf, len);
}

void SimpleShell::ProcessLine()
{
    char* argv[SHELL_MAX_ARGS];
    int argc = 0;
    char* token = strtok(_rxBuffer, " ");

    while (token != nullptr && argc < SHELL_MAX_ARGS)
    {
        argv[argc++] = token;
        token = strtok(nullptr, " ");
    }

    if (argc == 0) return;

    for (const auto& cmd : _commands)
    {
        if (strcmp(argv[0], cmd.name) == 0)
        {
            cmd.handler(argc, argv);
            return;
        }
    }
    Printf("Unknown command: %s\r\n", argv[0]);
}

void SimpleShell::PrintPrompt()
{
    Write(">> ", 3);
}

