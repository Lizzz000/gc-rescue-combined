#include "check.hpp"

void testMissionTrajectory();
void testMissionErrors();
void testRoiAndTransport();

int main()
{
  testMissionTrajectory();
  testMissionErrors();
  testRoiAndTransport();
  return finishTests("mission_pkg");
}
