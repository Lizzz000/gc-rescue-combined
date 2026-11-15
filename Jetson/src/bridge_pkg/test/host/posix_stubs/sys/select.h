#pragma once
#ifdef _WIN32
#include <time.h>
struct fd_set { bool set; };
#define FD_ZERO(p) ((p)->set = false)
#define FD_SET(fd,p) ((p)->set = true)
int select(int, fd_set *, fd_set *, fd_set *, timeval *);
#else
#include_next <sys/select.h>
#endif
