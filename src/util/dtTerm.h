/*!
\file       dtTerminal.h
\brief      Configure Terminal(Console) Attribute
\author     Dong-hyun Lee, phenom8305@gamil.com
\date       2021. 02. 05
\version    1.0.0
*/

#ifndef __DT_UTILS_TERMINAL_H__
#define __DT_UTILS_TERMINAL_H__

#include <cstdint>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>

/* ANSI Escape code standard color code */
#define COLOR_RESET 0
#define FG_BLACK 30
#define FG_RED 31
#define FG_GREEN 32
#define FG_YELLOW 33
#define FG_BLUE 34
#define FG_MAGENTA 35
#define FG_CYAN 36
#define FG_WHITE 37

#define BG_BLACK 40
#define BG_RED 41
#define BG_GREEN 42
#define BG_YELLOW 43
#define BG_BLUE 44
#define BG_MAGENTA 45
#define BG_CYAN 46
#define BG_WHITE 47

/* ANSI Escape code bright color code */
#define FG_BR_BRLACK 90
#define FG_BR_RED 91
#define FG_BR_GREEN 92
#define FG_BR_YELLOW 93
#define FG_BR_BLUE 94
#define FG_BR_MAGENTA 95
#define FG_BR_CYAN 96
#define FG_BR_WHITE 97

#define BG_BR_BLACK 100
#define BG_BR_RED 101
#define BG_BR_GREEN 102
#define BG_BR_YELLOW 103
#define BG_BR_BLUE 104
#define BG_BR_MAGENTA 105
#define BG_BR_CYAN 106
#define BG_BR_WHITE 107

/* ANSI Escape code */
#define CSI_ED_C2E "\x1b[0J" // clear screen cursor to end
#define CSI_ED_C2B "\x1b[1J" // clear screen cursor to begin
#define CSI_ED_A "\x1b[2J"   // clear screen entire
#define CSI_EL_C2E "\x1b[0K" // clear line cursor to end
#define CSI_EL_C2B "\x1b[1K" // clear line cursor to begin
#define CSI_EL_A "\x1b[2K"   // clear line entire
#define ESC_DECSC "\x1b7"    // saves the cursor position
#define ESC_DECRC "\x1b8"    // restores the cursor position
#define PPRINTF(row, col, fmt, ...)                         \
    do                                                      \
    {                                                       \
        printf("\x1b[%d;%dH" fmt, row, col, ##__VA_ARGS__); \
    } while (0)
#define CPRINTF(colorCode, fmt, ...)                                          \
    do                                                                        \
    {                                                                         \
        printf("\x1b[%dm" fmt "\x1b[0m", row, col, colorCode, ##__VA_ARGS__); \
    } while (0)
#define PCPRINTF(row, col, colorCode, fmt, ...)                                          \
    do                                                                                   \
    {                                                                                    \
        printf("\x1b[%d;%dH\x1b[%dm" fmt "\x1b[0m", row, col, colorCode, ##__VA_ARGS__); \
    } while (0)

namespace dt
{
namespace Utils
{
namespace Term
{
void SetupTerminal(bool showCursor = true);
void RestoreTerminal(void);
void GetCurPos(int *row, int *col);
#if !defined(_WIN32)
int kbhit(void);
#endif

static inline void ShowCursor(bool on)
{
    on ? system("setterm -cursor on") : system("setterm -cursor off");
}

static inline void SetColor(int code)
{
    printf("\x1b[%dm", code);
}

static inline void Set8BitColor(uint8_t code)
{
    printf("\x1b[38;5;%dm", code);
}

static inline void Set8BitFgColor(uint8_t r, uint8_t g, uint8_t b)
{
    // 0-7: standard colors
    // 8-15: high intensity colors
    // 16-231: 216 colors
    // 232-255: grayscale 24 steps

    if (r > 5 || g > 5 || b > 5) return;
    printf("\x1b[38;5;%dm", r * 36 + g * 6 + b + 16);
}

static inline void Set8BitBgColor(uint8_t r, uint8_t g, uint8_t b)
{
    // 0-7: standard colors
    // 8-15: high intensity colors
    // 16-231: 216 colors
    // 232-255: grayscale 24 steps

    if (r > 5 || g > 5 || b > 5) return;
    printf("\x1b[48;5;%dm", r * 36 + g * 6 + b + 16);
}

static inline void Set24BitFgColor(uint8_t r, uint8_t g, uint8_t b)
{
    printf("\x1b[38;2;%d;%d;%dm", r, g, b);
}

static inline void Set24BitBgColor(uint8_t r, uint8_t g, uint8_t b)
{
    printf("\x1b[48;2;%d;%d;%dm", r, g, b);
}

static inline void ResetColor(void)
{
    printf("\x1b[0m");
}

static inline void ClearDisp(void)
{
    printf("\x1b[2J");
}

static inline void ClearDispC2B(void)
{
    printf("\x1b[1J"); // clear screen from cursor to begin
}

static inline void ClearDispC2E(void)
{
    printf("\x1b[0J"); // clear screen from cursor to end
}

static inline void ClearLine(void)
{
    printf("\x1b[2K");
}

static inline void ClearLineC2B(void)
{
    printf("\x1b[1K");
}

static inline void ClearLineC2E(void)
{
    printf("\x1b[0K");
}

static inline void CurUp(int pos)
{
    printf("\x1b[%dA", pos);
}

static inline void CurDn(int pos)
{
    printf("\x1b[%dB", pos);
}

static inline void CurRt(int pos)
{
    printf("\x1b[%dC", pos);
}

static inline void CurLt(int pos)
{
    printf("\x1b[%dD", pos);
}

static inline void CurPos(int row, int col)
{
    printf("\x1b[%d;%dH", row, col); // cursor position
    // printf("\x1b[%d;%df", row, col); // cursor horizontal vertical position
}

static inline void SaveCurPos(void)
{
    printf("\x1b%d", 7);
}

static inline void RestoreCurPos(void)
{
    printf("\x1b%d", 8);
}

static inline void PrintTitle(const char *str)
{
    char title[66] = "=================================================================";
    size_t len = strlen(str);
    memcpy(&title[2], str, len);
    printf("%s\n", title);
}

static inline void PrintEndLine()
{
    char endBar[66] = "-----------------------------------------------------------------";
    printf("%s\n\n", endBar);
}

static inline void Print(const char *str)
{
    printf("%s", str);
    //fflush(stdout);
}

static inline int Printf(const char *fmt, ...)
{
    int rtn;
    va_list args;
    va_start(args, fmt);
    rtn = vprintf(fmt, args);
    va_end(args);
    //fflush(stdout);

    return rtn;
}

static inline int Printf(int r, int c, const char *fmt, ...)
{
    printf("\x1b[%d;%dH", r, c);
    va_list args;
    va_start(args, fmt);
    r = vprintf(fmt, args);
    va_end(args);
    //fflush(stdout);

    return r;
}

static inline int Printf(int color, const char *fmt, ...)
{
    printf("\x1b[%dm", color);
    va_list args;
    va_start(args, fmt);
    color = vprintf(fmt, args);
    va_end(args);
    printf("\x1b[0m");
    //fflush(stdout);

    return color;
}

static inline int Printf(int r, int c, int color, const char *fmt, ...)
{
    printf("\x1b[%d;%dH\x1b[%dm", r, c, color);
    va_list args;
    va_start(args, fmt);
    r = vprintf(fmt, args);
    va_end(args);
    printf("\x1b[0m");
    //fflush(stdout);

    return r;
}

static inline int Flush()
{
    return fflush(stdout);
}

} // namespace Term
} // namespace Utils
} // namespace dt

namespace dtTerm = ::dt::Utils::Term;

#endif // __DT_UTILS_TERMINAL_H__