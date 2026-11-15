#pragma once
#define LOCK_EX 1
#define LOCK_NB 2
int flock(int, int);
