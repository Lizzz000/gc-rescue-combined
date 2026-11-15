#pragma once
#define O_RDWR 1
#define O_NOCTTY 2
#define O_NONBLOCK 4
#define O_CLOEXEC 8
int open(const char *, int, ...);
