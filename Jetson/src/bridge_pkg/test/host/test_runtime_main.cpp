#include "check.hpp"
void testBridgeRuntimeSearch();
void testDeliveryLocalization();
int main()
{
  testBridgeRuntimeSearch();
  testDeliveryLocalization();
  return finishTests("BridgeRuntime");
}
