"""偏航 ADRC 初测的主机回归。

被控对象是 100 Hz 一阶偏航模型（增益、纯滞后、阻力死区、时段扰动与测量噪声），
用于对照当前固件阻尼实现和限幅 ADRC。这里的指标不是实车表现。
ADRC 仍未接入控制固件；固件增益与 results.json 中的初测参数均不在本测试中改写。
"""
import importlib.util
import json
import math
import unittest
from pathlib import Path

STM32 = Path(__file__).resolve().parents[1]
REPO = STM32.parent
SCRIPT = STM32 / "manual_test/live_tuning/adrc/yaw_adrc_pretest.py"
SPEC = importlib.util.spec_from_file_location("yaw_adrc_pretest", SCRIPT)
adrc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adrc)

RECORDED = json.loads((SCRIPT.with_name("results.json")).read_text(encoding="utf-8"))


class YawAdrcPretestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = adrc.load_config(REPO)
        cls.chosen = adrc.ObserverTuning(**RECORDED["tuning"])
        cls.chosen_p = adrc.ProportionalTuning(**RECORDED["retuned_proportional_tuning"])

    def test_load_config_matches_firmware_macros(self):
        self.assertEqual(self.cfg["CHASSIS_VECTOR_ACCEL_RAD_S2"], 1.6)
        self.assertEqual(self.cfg["CHASSIS_VECTOR_SMOOTHING_S"], 0.040)
        self.assertEqual(self.cfg["CHASSIS_YAW_RATE_FEEDBACK_KP"], 0.90)
        self.assertEqual(self.cfg["CHASSIS_YAW_RATE_FILTER_S"], 0.025)
        self.assertEqual(self.cfg["CHASSIS_YAW_RATE_DEADBAND_RAD_S"], 0.025)
        self.assertEqual(self.cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RAD_S"], 0.30)
        self.assertEqual(self.cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RATIO"], 0.40)

    def test_observer_recovers_constant_disturbance_and_rejects_bad_samples(self):
        adrc.observer_sanity(self.chosen)

    def test_zero_reference_forces_zero_command(self):
        self.assertEqual(adrc.bounded_yaw(0.0, 0.37, 0.3, 1.6, adrc.DT), 0.0)

    def test_same_direction_limit_and_slew(self):
        # 参考为正时不能越过参考，也不能反向；单步变化不超过加速度限制。
        applied = 0.10
        applied = adrc.bounded_yaw(0.32, 0.50, applied, 1.6, adrc.DT)
        self.assertGreaterEqual(applied, 0.0)
        self.assertLessEqual(applied, 0.32 + 1e-12)
        self.assertLessEqual(abs(applied - 0.10), 1.6 * adrc.DT + 1e-12)
        reversed_command = adrc.bounded_yaw(0.32, -0.20, 0.20, 1.6, adrc.DT)
        self.assertGreaterEqual(reversed_command, 0.0)
        self.assertLessEqual(reversed_command, 0.32 + 1e-12)

    def test_baseline_matches_firmware_reduction_example(self):
        # 与 test_chassis.c 中 0.40 指令、0.65 实测收到 0.24 的用例同一组宏。
        p = adrc.ProportionalTuning(self.cfg["CHASSIS_YAW_RATE_FEEDBACK_KP"],
                                    self.cfg["CHASSIS_YAW_RATE_FILTER_S"],
                                    self.cfg["CHASSIS_YAW_RATE_DEADBAND_RAD_S"])
        measured = 0.65
        reference = 0.40
        excess = measured - reference - p.deadband
        cap = min(self.cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RAD_S"],
                  self.cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RATIO"] * reference)
        reduction = min(cap, p.gain * max(0.0, excess))
        self.assertAlmostEqual(reduction, 0.16)
        self.assertAlmostEqual(reference - reduction, 0.24)

    def _quiet(self, name, model, speed=0.40, sign=1):
        return adrc.Scenario(name, speed, sign, *model, seed=1)

    def test_step_disturbance_saturation_and_brake(self):
        # 无测量噪声，便于直接看阶跃、扰动、限幅和停车。
        nominal = self._quiet("nominal", (1.12, 0.080, 2, 0.012, 0.0, 0.0))
        disturbed = self._quiet("disturb", (1.12, 0.080, 2, 0.012, 2.0, 0.0))
        loaded = self._quiet("loaded", (0.88, 0.160, 4, 0.045, -1.8, 0.0))
        cases = {
            "nominal": (nominal, adrc.compare_run(nominal, self.cfg),
                        adrc.compare_run(nominal, self.cfg, self.chosen)),
            "disturb": (disturbed, adrc.compare_run(disturbed, self.cfg),
                        adrc.compare_run(disturbed, self.cfg, self.chosen)),
            "loaded": (loaded, adrc.compare_run(loaded, self.cfg),
                       adrc.compare_run(loaded, self.cfg, self.chosen)),
        }
        for name, (_, baseline, experimental) in cases.items():
            with self.subTest(name=name):
                for row in (baseline, experimental):
                    self.assertTrue(math.isfinite(row["rmse"]))
                    self.assertGreaterEqual(row["max_overspeed"], 0.0)
                    self.assertLessEqual(row["max_slew"],
                                         self.cfg["CHASSIS_VECTOR_ACCEL_RAD_S2"] + 1e-6)
                    self.assertIsNotNone(row["settle_s"])
                    self.assertLessEqual(row["settle_s"], 0.80)
                print(f"host plant {name}: baseline rmse={baseline['rmse']:.4f} "
                      f"settle={baseline['settle_s']:.3f}s; "
                      f"ADRC rmse={experimental['rmse']:.4f} "
                      f"settle={experimental['settle_s']:.3f}s "
                      f"(simulation only)")

    def test_recorded_metrics_reproduced_without_retuning(self):
        evaluated = adrc.evaluate(self.cfg, self.chosen, self.chosen_p)
        self.assertEqual(evaluated["training_count"], RECORDED["training_count"])
        self.assertEqual(evaluated["holdout_count"], RECORDED["holdout_count"])
        self.assertEqual(evaluated["cases_with_higher_rmse"], RECORDED["cases_with_higher_rmse"])
        self.assertEqual(evaluated["cases_with_higher_rmse_than_retuned_p"],
                         RECORDED["cases_with_higher_rmse_than_retuned_p"])
        for key in ("training_adrc", "training_retuned_proportional", "holdout_baseline",
                    "holdout_adrc", "holdout_retuned_proportional"):
            for path, actual, expected in adrc.metric_pairs(evaluated[key], RECORDED[key]):
                self.assertAlmostEqual(actual, expected, delta=1e-12, msg=f"{key}.{path}")
        self.assertAlmostEqual(evaluated["worst_rmse_increase"],
                               RECORDED["worst_rmse_increase"], delta=1e-12)


if __name__ == "__main__":
    unittest.main()
