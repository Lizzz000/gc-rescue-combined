#pragma once
struct stat { unsigned st_mode; };
#define S_ISCHR(mode) ((mode) == 1)
int fstat(int, struct stat *);
