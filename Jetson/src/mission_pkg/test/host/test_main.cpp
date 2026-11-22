#include "check.hpp"

void testMissionTrajectory();
void testMissionErrors();
void testMissionRoiIdentity();
void testSelectedPickupRoute();
void testRoiAndTransport();
void testFieldGeometry();

int main()
{
  testMissionTrajectory();
  testMissionErrors();
  testMissionRoiIdentity();
  testSelectedPickupRoute();
  testRoiAndTransport();
  testFieldGeometry();
  return finishTests("mission_pkg");
}
