
// 阅读：--role 为 mission、target 或 safety，各启动一个节点。
// mission 用 control/controller.hpp 编排阶段，并请求 target 的 ROI 和检测端切换。
// target 转发目标并检查原始检测。safety 把场地观测收成目的区中心。
// 下一份：node/mission_node.hpp。规则在 control/controller.hpp 与 roi/roi.hpp。
// ---- 角色入口与组件工厂：与节点共用一份生产对象 ----
#include "node/mission_node.hpp"
#include "node/target_node.hpp"
#include "node/safety_node.hpp"

int main(int argc, char ** argv)
{
  return rescue::dispatchRuntime(argc, argv,
    {{"mission", rescue::runNodeRole<MissionNode>},
      {"target", rescue::runNodeRole<TargetNode>},
      {"safety", rescue::runNodeRole<SafetyNode>}});
}
