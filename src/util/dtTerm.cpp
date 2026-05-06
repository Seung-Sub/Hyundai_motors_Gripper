#include "util/dtTerm.h"

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

namespace dt
{
namespace Utils
{


static struct termios oriTerm;
static struct termios newTerm;

void Term::SetupTerminal(bool showCursor)
{
    tcgetattr(STDIN_FILENO, &oriTerm);
    newTerm = oriTerm;
    newTerm.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newTerm);
    int res = 0;
    res = showCursor ? system("setterm -cursor on") : system("setterm -cursor off");
}

void Term::RestoreTerminal(void)
{
    // Reset colors
    printf("\x1b[0m");

    // Reset console mode
    tcsetattr(STDIN_FILENO, TCSANOW, &oriTerm);

    int res = 0;
    res = system("setterm -cursor on");
}

int Term::kbhit(void)
{
    int ch;
    int oldFd;

    oldFd = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldFd | O_NONBLOCK);
    ch = getchar();
    fcntl(STDIN_FILENO, F_SETFL, oldFd);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void Term::GetCurPos(int *row, int *col)
{
    char buff[128];
    int indx = 0;

    printf("\x1b[6n");

    for (;;)
    {
        int cc = getchar();
        buff[indx] = (char)cc;
        indx++;
        if (cc == 'R')
        {
            buff[indx + 1] = '\0';
            break;
        }
    }

    sscanf(buff, "\x1b[%d;%dR", row, col);
    fseek(stdin, 0, SEEK_END);
}

} // namespace Utils
} // namespace dt
