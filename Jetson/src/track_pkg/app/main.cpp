// 阅读：--role track 订阅检测并发布轨迹；select 按任务资格选出一个目标。
// 两个角色都由本进程二选一。这里不推理，也不发 CAN。
// 下一份：node/track_node.hpp、node/select_node.hpp。算法在 tracking/tracking.hpp。
#include <interface_pkg/app/core.hpp>
#include "node/track_node.hpp"
#include "node/select_node.hpp"

/**
 * @brief 在统一入口选择track或select角色，创建对应单节点进程。
 * @param[in] argc 命令行参数个数，非负。
 * @param[in] argv 参数数组；argc个有效项后须以空指针结尾。
 * @return 0表示正常退出，2表示配置错误，1表示运行失败。
 */
int main(int argc, char ** argv)
{
  return rescue::dispatchRuntime(
    argc, argv,
    {{"track", rescue::runNodeRole<TrackNode>}, {"select", rescue::runNodeRole<SelectNode>}});
}
