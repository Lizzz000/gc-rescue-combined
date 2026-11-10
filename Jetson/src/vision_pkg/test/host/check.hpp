#pragma once

#include <cstdio>

// 每条 CHECK 是一条断言。分组在析构时打印，方便对上基线报告。
inline int g_passed = 0;
inline int g_failed = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (static_cast<bool>(cond)) {                                           \
      ++g_passed;                                                            \
    } else {                                                                 \
      ++g_failed;                                                            \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    }                                                                        \
  } while (0)

struct Group {
  const char * name;
  int passed_before;
  int failed_before;

  explicit Group(const char * group_name)
  : name(group_name), passed_before(g_passed), failed_before(g_failed) {}

  ~Group()
  {
    std::printf(
      "%s: %d passed, %d failed\n",
      name, g_passed - passed_before, g_failed - failed_before);
  }
};

inline int finishTests(const char * suite)
{
  std::printf("%s total: %d passed, %d failed\n", suite, g_passed, g_failed);
  return g_failed == 0 ? 0 : 1;
}
