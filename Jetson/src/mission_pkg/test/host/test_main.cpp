#include "check.hpp"

void testMissionTrajectory();
void testMissionErrors();
void testRoiAndTransport();
void testFieldGeometry();

int main()
{
  testMissionTrajectory();
  testMissionErrors();
  testRoiAndTransport();
  testFieldGeometry();
  return finishTests("mission_pkg");
}
