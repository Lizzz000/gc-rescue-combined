#include "check.hpp"

#include <cstring>

void testColourClassification();
void testFieldRobustness(bool report);

int main(int argc, char ** argv)
{
  // --report 打印逐场景统计表，供调参与回归对比
  const bool report = argc > 1 && std::strcmp(argv[1], "--report") == 0;
  testColourClassification();
  testFieldRobustness(report);
  return finishTests("vision_pkg field");
}
