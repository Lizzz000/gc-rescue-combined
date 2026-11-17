# AHRS

正式任务调用 `AHRS_init`、`AHRS_update`、`get_angle`。保留Mahony六轴/九轴反馈公式、现有增益、安装矩阵和磁场/陀螺标定；四元数w/x/y/z表示机体系到世界系，角度单位rad。

本轮在封装入口拒绝非法四元数、陀螺和非正/超过50ms的dt，保留最后姿态；无效加速度按无重力纠正处理，无效磁场回退六轴。无效重力不能写入启动姿态。欧拉角使用齐次表达式并复用平方项，减少归一化误差影响；无效四元数输出NaN供上层判无效。直接调用底层 `MahonyAHRSupdate*` 仍须遵守其有效输入约定。

参考 [Mahony等2008年论文](https://scispace.com/papers/nonlinear-complementary-filters-on-the-special-orthogonal-ysp0zzaoar)以及 [RoboMaster固定源版本](https://github.com/RoboMaster/Development-Board-C-Examples/blob/bd594e1aa22c8e4c5d5895a8328050fda21186d8/18.ins_task/components/algorithm/MahonyAHRS.c)。GPL声明继续保留。这里没有改成EKF，也没有未经标定地开启陀螺积分校正。

六轴航向仍会漂移；有限数值与稳定单位四元数不等于绝对航向准确。主机测试覆盖已知姿态100秒、极点、时步变化、磁场缺失、非法输入和小幅四元数缩放。PC基准与板端验证范围见 [本轮记录](../../../../docs/ALGORITHM_UPGRADE_20261117.md)。
