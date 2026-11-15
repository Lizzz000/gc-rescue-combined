#pragma once
#include <cstddef>
#include <sys/types.h>
int close(int);
ssize_t read(int, void *, std::size_t);
ssize_t write(int, const void *, std::size_t);
