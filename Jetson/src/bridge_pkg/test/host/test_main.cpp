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
void testBridgeTransactions();

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
  testBridgeTransactions();
  return finishTests("bridge_pkg");
}
