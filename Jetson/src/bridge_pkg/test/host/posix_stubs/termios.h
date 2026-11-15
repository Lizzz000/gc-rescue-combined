#pragma once
struct termios { unsigned c_cflag; unsigned char c_cc[4]; };
#define CLOCAL 1
#define CREAD 2
#define VMIN 0
#define VTIME 1
#define TCSANOW 0
int tcgetattr(int, termios *);
void cfmakeraw(termios *);
int tcsetattr(int, int, const termios *);
