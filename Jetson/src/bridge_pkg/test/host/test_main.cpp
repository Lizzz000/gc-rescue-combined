#include "check.hpp"

void testCanProtocol();
void testCanTxQueue();
void testBridgeState();

int main()
{
  testCanProtocol();
  testCanTxQueue();
  testBridgeState();
  return finishTests("bridge_pkg");
}
