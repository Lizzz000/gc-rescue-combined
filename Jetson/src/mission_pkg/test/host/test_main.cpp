#include "check.hpp"

void testMissionTrajectory();
void testMissionErrors();
void testMissionRoiIdentity();
void testRoiAndTransport();
void testFieldGeometry();

int main()
{
  testMissionTrajectory();
  testMissionErrors();
  testMissionRoiIdentity();
  testRoiAndTransport();
  testFieldGeometry();
  return finishTests("mission_pkg");
}
