#include "check.hpp"

void testCanProtocol();
void testCanTxQueue();
void testBridgeState();
void testSafetyHold();

int main()
{
  testCanProtocol();
  testCanTxQueue();
  testBridgeState();
  testSafetyHold();
  return finishTests("bridge_pkg");
}
