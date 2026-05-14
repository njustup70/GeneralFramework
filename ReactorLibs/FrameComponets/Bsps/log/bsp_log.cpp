#include "bsp_log.hpp"
#include "bsp_uart.hpp"
#include "SEGGER_RTT.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "bsp_hardware.hpp"

typedef struct {
    const char* cmd_name;    // 命令名，如 "mdji"
    cmd_handler_t handler;   // 对应的回调函数
    const char* help;        // 帮助信息
} shell_cmd_t;

const _RTT_Color_t RTT_Color = {
    .RESET  = "\x1B[0m",
    .RED    = "\x1B[31m",
    .GREEN  = "\x1B[32m",
    .YELLOW = "\x1B[33m",
    .BLUE   = "\x1B[34m"
};

#define RTT_GREEN RTT_CTRL_TEXT_BRIGHT_GREEN
#define RTT_YELLOW RTT_CTRL_TEXT_BRIGHT_YELLOW
#define RTT_RED RTT_CTRL_TEXT_BRIGHT_RED
#define RTT_RESET RTT_CTRL_RESET
#define RTT_BLUE RTT_CTRL_TEXT_BRIGHT_BLUE
#define RTT_PURPLE RTT_CTRL_TEXT_BRIGHT_MAGENTA

/// @brief 命令接受缓冲区大小
#define RTTRX_BUFSIZE 64
static char rttrx_buf[RTTRX_BUFSIZE];
static uint8_t rttrx_idx = 0;
static void Process_Command(char *cmd_str);

/// @brief 命令表
#define MAX_SHELL_CMDS 24
static shell_cmd_t cmd_table[MAX_SHELL_CMDS];
static uint8_t cmd_count = 0;

// 提前获取 UART 句柄，用于后续日志输出
BSP::UART::Handler uart_log_handler;

void BspLog_Init(void)
{
    SEGGER_RTT_Init();
    
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    
    // 先打一个简单的字符串确认链路
    BspLog_LogOK("RTT System Online\r\n");

    // 如果确定使用 UART 输出日志，可以在这里获取 huart_host 的句柄
    if (Hardware::RTTLogAtUart)
    {
        if (Hardware::RTTLogUartItrp)
        {
            uart_log_handler = BSP::UART::Apply(Hardware::huart_log, BSP::UART::TxMode::Interrupt);
        }
        else
        {
            uart_log_handler = BSP::UART::Apply(Hardware::huart_log);
        }
    }
}

/**
 * @warning 串口日志发送
 */
static char log_tx_buf[96]; // 稍微加大一点缓冲区防止长日志截断
static void LogToUart(const char* prefix, const char* fmt, va_list args)
{
    if (Hardware::RTTLogAtUart && uart_log_handler.IsValid())
    {
        int len = 0;
        const int max_len = (int)sizeof(log_tx_buf) - 1; // 留一个位置给 \0
        
        if (prefix)
        {
            int ret = snprintf(log_tx_buf + len, max_len - len + 1, "%s", prefix);
            if (ret > 0)
            {
                int available = max_len - len;
                len += (ret > available) ? available : ret;
            }
        }
        
        if (len < max_len)
        {
            int ret = vsnprintf(log_tx_buf + len, max_len - len + 1, fmt, args);
            if (ret > 0)
            {
                int available = max_len - len;
                len += (ret > available) ? available : ret;
            }
        }

        if (len > 0)
        {
            if (log_tx_buf[len - 1] != '\n')
            {
                if (len < max_len)
                {
                    log_tx_buf[len++] = '\n';
                    log_tx_buf[len] = '\0';
                }
                else
                {
                    log_tx_buf[len - 1] = '\n';
                }
            }

            // BspUart_Transmit_DMA(Hardware::huart_host, (uint8_t*)log_tx_buf, (uint8_t)len);
            uart_log_handler.Transmit((uint8_t*)log_tx_buf, (uint8_t)len);
        }
    }
}

void BspLog_LogInfo(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);

    va_start(args, fmt);
    LogToUart("[Info] ", fmt, args); 
    va_end(args);
}

void BspLog_LogWarning(const char* fmt, ...)
{
    va_list args;
    SEGGER_RTT_WriteString(0, RTT_YELLOW "[Warn] ");
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    SEGGER_RTT_WriteString(0, RTT_RESET);

    va_start(args, fmt);
    LogToUart("[Warn] ", fmt, args);
    va_end(args);
}

void BspLog_LogError(const char* fmt, ...)
{
    va_list args;
    SEGGER_RTT_WriteString(0, RTT_RED "[Error] ");
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    SEGGER_RTT_WriteString(0, RTT_RESET);

    va_start(args, fmt);
    LogToUart("[Error] ", fmt, args);
    va_end(args);
}

void BspLog_LogOK(const char* fmt, ...)
{
    va_list args;
    SEGGER_RTT_WriteString(0, RTT_GREEN "[Well] ");
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    SEGGER_RTT_WriteString(0, RTT_RESET);

    va_start(args, fmt);
    LogToUart("[Well] ", fmt, args);
    va_end(args);
}

void BspLog_LogSpec(const char* fmt, ...)
{
    va_list args;
    SEGGER_RTT_WriteString(0, RTT_PURPLE "[Note] ");
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    SEGGER_RTT_WriteString(0, RTT_RESET);

    va_start(args, fmt);
    LogToUart("[Note] ", fmt, args);
    va_end(args);
}

void BspLog_LogRespond(const char* fmt, ...)
{
    va_list args;
    SEGGER_RTT_WriteString(0, RTT_BLUE "[Respond] ");
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    SEGGER_RTT_WriteString(0, RTT_RESET);

    va_start(args, fmt);
    LogToUart("[Respond] ", fmt, args);
    va_end(args);
}


/**
 * @brief 处理接收到的命令
 * @note 本函数需要循环调用以检查 RTT 命令
 */
void BspLog_RecvCMD(void)
{
    char c;
    // 尝试从 RTT 通道 0 读取字节
    // 返回值是读取到的字节数，0 表示没有数据
    while (SEGGER_RTT_Read(0, &c, 1) > 0)
    {
        // 只有不是换行符时，才存入 buffer
        if (c != '\n' && c != '\r')
        {
            if (rttrx_idx < RTTRX_BUFSIZE - 1)
            {
                rttrx_buf[rttrx_idx++] = c; // 存入
            }
        } 
        else 
        {
            // 收到换行符，说明指令结束
            if (rttrx_idx > 0) {
                rttrx_buf[rttrx_idx] = '\0';    // 补上字符串结束符
                Process_Command(rttrx_buf);     // 触发解码逻辑
                rttrx_idx = 0;                  // 索引归零，准备接收下一条
            }
        }
    }
}

/**
 * @brief 由外界调用以注册命令
 */
void BspLog_RegistCMD(const char* name, cmd_handler_t handler, const char* help) 
{
    if (cmd_count < MAX_SHELL_CMDS) 
    {
        cmd_table[cmd_count].cmd_name = name;
        cmd_table[cmd_count].handler = handler;
        cmd_table[cmd_count].help = help;
        cmd_count++;
    }
    else
    {
        BspLog_LogError("Too Many CMDs Registered, %s failed\n", name);
    }
}


static void Process_Command(char *cmd_str)
{
    char *argv[16];
    int argc = 0;

    // 使用空格作为分隔符，将字符串切碎
    // 第一段是命令名，后续是参数
    char *token = strtok(cmd_str, " ");

    while (token != NULL && argc < 16)
    {
        argv[argc++] = token;       // 记录当前参数的指针
        token = strtok(NULL, " ");  // 获取下一个
    }

    // 如果只是按了回车，没有任何字符，直接返回
    if (argc == 0) return;

    // 匹配接收到的命令
    for (uint8_t i = 0; i < cmd_count; i++)
    {
        // 比较输入的命令名 (argv[0]) 与表中的 cmd_name
        if (strcmp(argv[0], cmd_table[i].cmd_name) == 0)
        {
            // 执行回调
            if (cmd_table[i].handler != NULL)
            {
                // 将 argc 和 argv 传给具体的业务函数
                cmd_table[i].handler(argc, argv);
            }
            return; // 找到并执行后，立即结束
        }
    }

    // 遍历完所有注册命令都没找到匹配项
    BspLog_LogWarning("%s is not a CMD\r\n", argv[0]);
}