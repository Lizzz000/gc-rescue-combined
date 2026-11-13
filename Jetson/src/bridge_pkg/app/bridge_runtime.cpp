// 阅读：只启动 BridgeNode。它订阅目标和安全区，经 USB 交给 STM32。
// 到达后调用 mission 的 ROI 与命令服务。
// 下一份：node/bridge_node.hpp。状态在 state/bridge_state.hpp，帧格式在 protocol/usb_protocol.hpp。
// ---- 角色入口：与节点共用一份生产对象 ----
#include "node/bridge_node.hpp"
#include <interface_pkg/app/core.hpp>

/**
 * @brief 在程序入口解析角色，再交由统一 ROS 生命周期运行对应节点。
 * @param[in] argc 命令行参数数量，包含程序名，正常启动时至少为 1。
 * @param[in] argv 非空参数数组，前 argc 项为非空字符串，末项为空指针。
 * @return 0 表示正常结束，2 表示配置被拒绝，1 表示运行异常；失败不启动其他角色。
 */
int main(int argc, char ** argv)
{
  return rescue::dispatchRuntime(argc, argv,
    {{"bridge", rescue::runNodeRole<BridgeNode>}});
}
