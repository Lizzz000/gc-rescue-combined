#include "check.hpp"

void testCanProtocol();
void testCanTxQueue();
void testBridgeState();
void testSafetyHold();
void testArrivalPoint();

int main()
{
  testCanProtocol();
  testCanTxQueue();
  testBridgeState();
  testSafetyHold();
  testArrivalPoint();
  return finishTests("bridge_pkg");
}
