#pragma once
#define TIOCEXCL 1
#define TIOCNXCL 2
int ioctl(int, unsigned long, ...);
