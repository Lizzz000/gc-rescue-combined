#include "check.hpp"

void testCanProtocol();
void testFrameAlgorithm();
void testCanTxQueue();
void testBridgeState();
void testSafetyHold();
void testSearchReplyOrdering();
void testArrivalPoint();
void testMissionRestart();
void testSearchRequest();

int main()
{
  testCanProtocol();
  testFrameAlgorithm();
  testCanTxQueue();
  testBridgeState();
  testSafetyHold();
  testSearchReplyOrdering();
  testArrivalPoint();
  testMissionRestart();
  testSearchRequest();
  return finishTests("bridge_pkg");
}
