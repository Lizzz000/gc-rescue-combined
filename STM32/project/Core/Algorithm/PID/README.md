# PID

`PID_calc` 保留按调用次数的温控离散增益；`PID_Step` 使用显式dt、连续积分及测量微分。两者使用独立实例，不能互换Ki单位。正常输入的输出限幅、积分限幅和同向饱和抗积分规律保持。

本轮复用公共限幅实现，补充初始化、NaN/Inf、非正dt、非法限幅与中间算术溢出检查。错误输入返回0并清除历史，避免后续控制被NaN污染。温控单向限幅 `min_out=0` 继续支持；未修改温度设定、增益或电机PID参数。

来源为 [RoboMaster固定版本PID](https://github.com/RoboMaster/Development-Board-C-Examples/blob/bd594e1aa22c8e4c5d5895a8328050fda21186d8/20.standard_robot/components/controller/pid.c)，以及本项目已授权旧车的连续时间PID。保留GPL声明。

主机验证离散输出回归、变dt积分一致、无目标阶跃微分冲击、正负/单向饱和恢复和非法输入恢复。新增防护有计算成本，PC基准中单次约增加3ns；不能宣称每个函数都更快。STM32周期与闭环表现须待板端确认，见 [本轮记录](../../../../docs/ALGORITHM_UPGRADE_20261117.md)。
