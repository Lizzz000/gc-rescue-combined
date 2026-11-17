"""Offline tests of route generation, actual C estimator and independent plant."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "tools/live_tuning/simulate_return.py"
SPEC = importlib.util.spec_from_file_location("simulate_return", SCRIPT)
sim = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = sim
SPEC.loader.exec_module(sim)
ANALYSIS_SPEC = importlib.util.spec_from_file_location("analyze_runs", SCRIPT.with_name("analyze_runs.py"))
analysis = importlib.util.module_from_spec(ANALYSIS_SPEC)
ANALYSIS_SPEC.loader.exec_module(analysis)


class ReturnSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory(prefix="return_simulation_")
        cls.native = sim.NativeOdometry(cls.folder.name)

    @classmethod
    def tearDownClass(cls):
        cls.native.close()
        cls.folder.cleanup()

    def test_route_reproducible_bounded_and_has_transport_turns_and_dwell(self):
        route = sim.make_route()
        self.assertEqual(route, sim.make_route())
        self.assertNotEqual(route, sim.make_route(seed=1))
        moves = [a for a in route["actions"] if a["kind"] == "transport"]
        self.assertEqual(len(moves), 8)
        self.assertEqual({a["direction"] for a in moves}, {"forward", "backward", "left", "right"})
        self.assertEqual({a["kind"] for a in route["actions"]}, {"transport", "turn", "dwell"})
        bound = route["box_side_m"] / 2 - route["nominal_margin_m"]
        self.assertTrue(all(abs(a["x_m"]) <= bound and abs(a["y_m"]) <= bound for a in route["actions"]))
        for kwargs in ({"segments": 9}, {"speed_mps": .101}, {"box_m": .51}):
            with self.assertRaises(ValueError):
                sim.make_route(**kwargs)

    def test_return_vector_uses_current_body_heading(self):
        bx, by = sim.body_vector(-.1, -.2, math.pi / 2)
        self.assertAlmostEqual(bx, -.2)
        self.assertAlmostEqual(by, .1)

    def test_confirmed_wheel_layout_and_physical_lateral_feedback(self):
        # Rear, right front, left front contacts in the forward/left body frame.
        contacts = ((-.0982, 0.), (.0491, -.0491 * math.sqrt(3)), (.0491, .0491 * math.sqrt(3)))
        for vx, vy, wz in ((.2, 0., 0.), (0., .2, 0.), (0., 0., .5), (.15, -.1, -.3)):
            wheels = [((-y / .0982) * (vx - wz * y) + (x / .0982) * (vy + wz * x)) / .035
                      for x, y in contacts]
            with self.subTest(vx=vx, vy=vy, wz=wz):
                for actual, expected in zip(sim.inverse(vx, vy, wz), wheels):
                    self.assertAlmostEqual(actual, expected, places=6)
                for actual, expected in zip(sim.forward(wheels), (vx, vy, wz)):
                    self.assertAlmostEqual(actual, expected, places=6)
                motor = analysis.motor_targets(vx, vy, wz, rotation_radius=sim.OFFSET)
                for actual, expected in zip(motor, wheels):
                    self.assertAlmostEqual(actual, -expected, places=6)
                for actual, expected in zip(analysis.target_translation(motor), (vx, vy)):
                    self.assertAlmostEqual(actual, expected, places=6)
        # Known leftward contact speeds bypass both inverse implementations.
        left_wheels = [-.2 / .035, .1 / .035, .1 / .035]
        self.native.reset()
        for step in range(101):
            pose = self.native.update([math.remainder(w * step * .01, 25.) for w in left_wheels],
                                      left_wheels, 0., step * .01)
        self.assertAlmostEqual(pose[0], 0., places=6)
        self.assertAlmostEqual(pose[1], .2, places=6)

    def test_feedback_comes_from_motor_state_not_target(self):
        plant = sim.Plant(sim.IDEAL, 3)
        target = sim.inverse(.1, 0, 0)
        plant.step((.1, 0, 0), False)
        positions, feedback, _ = plant.feedback()
        self.assertNotEqual(feedback, target)
        self.assertLess(abs(feedback[1]), abs(target[1]))
        self.assertGreater(plant.x, 0)
        self.assertLess(plant.x, .1 * sim.DT)
        self.assertNotEqual(positions, [0.] * 3)

    def test_stationary_encoder_does_not_integrate_velocity_feedback_tail(self):
        self.native.reset()
        for step in range(101):
            pose = self.native.update([12.49, -12.49, .3], [.1, -.2, -.3], 0., step * .01)
        self.assertEqual(pose[0], 0.)
        self.assertEqual(pose[1], 0.)
        self.assertNotEqual(pose[3], 0.)  # Diagnostic measured velocity is still visible.

    def test_ideal_closes_estimated_and_independent_ground_pose(self):
        result, trace = sim.run_scenario(sim.make_route(), sim.IDEAL, self.native)
        self.assertEqual(result["status"], "completed")
        self.assertLess(result["estimated_origin_error_m"], .003)
        self.assertTrue(result["estimated_return_within_tolerance"])
        self.assertTrue(result["true_return_within_tolerance"])
        self.assertLess(result["true_origin_error_m"], .0035)
        self.assertLess(abs(result["true_final_yaw_deg"]), .3)
        self.assertTrue(all(math.hypot(r["cmd_vx_mps"], r["cmd_vy_mps"]) <= .10000001 for r in trace))
        self.assertTrue(all(abs(r["cmd_wz_rad_s"]) <= .3 for r in trace))

    def test_biased_feedback_and_slip_do_not_masquerade_as_true_return(self):
        route = sim.make_route()
        result, trace = sim.run_scenario(route, sim.PERTURBED, self.native)
        repeat, repeated_trace = sim.run_scenario(route, sim.PERTURBED, self.native)
        self.assertEqual(result, repeat)
        self.assertEqual(trace, repeated_trace)
        self.assertEqual(result["status"], "completed")
        self.assertLess(result["estimated_origin_error_m"], .02)
        # Equal distances from the origin can still represent different poses.
        # Check vector disagreement, and compute each verdict from its own pose.
        self.assertGreater(result["max_estimate_ground_disagreement_m"], .002)
        endpoint = trace[-1]
        self.assertAlmostEqual(result["true_origin_error_m"],
                               math.hypot(endpoint["true_x_m"], endpoint["true_y_m"]))
        self.assertAlmostEqual(result["estimated_origin_error_m"],
                               math.hypot(endpoint["estimated_x_m"], endpoint["estimated_y_m"]))
        self.assertGreater(abs(result["true_final_yaw_deg"]), .3)
        self.assertTrue(result["estimated_return_within_tolerance"])
        self.assertEqual(result["true_return_within_tolerance"],
                         result["true_origin_error_m"] <= .02 and abs(result["true_final_yaw_deg"]) <= 5)
        self.assertIn("source_sha256", self.native.provenance)

    def test_excessive_heading_drift_can_pass_estimate_and_fail_ground_verdict(self):
        assumed = sim.Model("test_excessive_drift", imu_drift_deg_s=.3)
        result, _ = sim.run_scenario(sim.make_route(), assumed, self.native)
        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["estimated_return_within_tolerance"])
        self.assertFalse(result["true_return_within_tolerance"])
        self.assertGreater(abs(result["true_final_yaw_deg"]), 5)


if __name__ == "__main__":
    unittest.main()
