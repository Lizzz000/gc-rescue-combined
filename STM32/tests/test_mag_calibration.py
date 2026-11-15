"""独立合成椭球与第二轮方向数据，验证电脑端磁标定求解和失败行为。"""
import importlib.util
import io
from contextlib import redirect_stderr
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "tools/mag_calibration/fit_magnetometer.py"
SPEC = importlib.util.spec_from_file_location("mag_fit", SCRIPT)
mag_fit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mag_fit)


def sphere_points(count, phase=0):
    """Fibonacci球面方向；测试数据不由生产拟合器生成。"""
    i = np.arange(count)
    z = 1 - 2*(i + .5)/count
    angle = i * np.pi * (3-np.sqrt(5)) + phase
    r = np.sqrt(1-z*z)
    return np.column_stack((r*np.cos(angle), r*np.sin(angle), z))


class MagnetometerFitTests(unittest.TestCase):
    def setUp(self):
        self.bias = np.array([11.0, -7.0, 13.0])
        # 非对角SPD矩阵：不能靠三轴独立缩放通过测试。
        axis = np.array([1., 2., 3.]) / np.sqrt(14)
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotation = np.eye(3) + np.sin(.7)*cross + (1-np.cos(.7))*(cross@cross)
        self.true_matrix = rotation @ np.diag([1.25, .85, 1.05]) @ rotation.T
        self.field = 49.0

    def measured(self, direction):
        return np.linalg.solve(self.true_matrix, (direction*self.field).T).T + self.bias

    def test_rotated_ellipsoid_and_independent_validation(self):
        train = self.measured(sphere_points(1800))
        bias, matrix, radius = mag_fit.fit(train)
        np.testing.assert_allclose(bias, self.bias, atol=1e-7)
        # 固定R来自球拟合，矩阵只可与真实矩阵在同一半径尺度比较。
        np.testing.assert_allclose(matrix*self.field/radius, self.true_matrix, atol=1e-8)
        check_directions = sphere_points(1100, phase=.83)
        check = self.measured(check_directions)
        corrected = mag_fit.apply_calibration(check, bias, matrix)
        np.testing.assert_allclose(corrected/radius, check_directions, atol=1e-8)
        result = mag_fit.statistics(check, bias, matrix, radius)
        self.assertLess(result["corrected"]["relative_std"], 1e-9)
        self.assertGreater(result["raw"]["relative_std"], .1)
        self.assertTrue(all(result["coverage"]["octant_counts_xyz_bits"]))
        twice = mag_fit.apply_calibration(corrected, bias, matrix)
        self.assertGreater(np.std(np.linalg.norm(twice, axis=1))/radius, .1)

    def test_analytic_jacobians_against_central_difference(self):
        rng = np.random.default_rng(20261115)
        max_error = 0.0
        for _ in range(20):
            points = rng.normal(size=(40, 3))
            offset = rng.normal(size=3) * .2
            basis = rng.normal(size=(3, 3))
            matrix = basis.T @ basis + np.eye(3)*.5
            sphere = np.r_[1., offset]
            ellipsoid = np.r_[offset, np.diag(matrix), matrix[0, 1], matrix[0, 2], matrix[1, 2]]
            cases = ((sphere, mag_fit.sphere_jacobian,
                      lambda p: p[0] - np.linalg.norm(points-p[1:], axis=1)),
                     (ellipsoid, mag_fit.ellipsoid_jacobian,
                      lambda p: 1.0 - np.linalg.norm((points-p[:3]) @
                          mag_fit.matrix_from_parameters(p[3:]).T, axis=1)))
            for parameters, jacobian, residual in cases:
                finite = np.empty((len(points), len(parameters)))
                for column in range(len(parameters)):
                    step = np.zeros_like(parameters)
                    step[column] = 1e-5
                    finite[:, column] = (residual(parameters+step)-residual(parameters-step))/(2e-5)
                actual = jacobian(parameters, points)
                max_error = max(max_error, float(np.max(np.abs(actual-finite))))
                np.testing.assert_allclose(actual, finite, atol=1e-7, rtol=1e-7)
        print(f"Jacobian central difference max abs error: {max_error:.3g}")
        # 位于中心的单点没有可定义方向；对称差分为零，不污染其它方向。
        np.testing.assert_array_equal(mag_fit.sphere_jacobian(np.zeros(4), np.zeros((1, 3))), [[1, 0, 0, 0]])
        np.testing.assert_array_equal(mag_fit.ellipsoid_jacobian(
            np.r_[np.zeros(3), np.ones(3), np.zeros(3)], np.zeros((1, 3))), np.zeros((1, 9)))

    def test_same_recorded_data_as_finite_difference_solver(self):
        folder = SCRIPT.parent / "data/2026-11-05"
        train = mag_fit.read_samples(folder / "fit.csv")
        check = mag_fit.read_samples(folder / "validation.csv")
        solver = mag_fit.least_squares
        results, counts = [], []
        for numerical in (True, False):
            count = [0]

            def instrument(residual, *args, **kwargs):
                if numerical:
                    kwargs["jac"] = "2-point"

                def counted(parameters):
                    count[0] += 1
                    return residual(parameters)

                return solver(counted, *args, **kwargs)

            with patch.object(mag_fit, "least_squares", side_effect=instrument):
                results.append(mag_fit.fit(train))
            counts.append(count[0])
        for before, after in zip(results[0], results[1]):
            np.testing.assert_allclose(before, after, rtol=1e-7, atol=1e-5)
        for samples in (train, check):
            before, after = [mag_fit.statistics(samples, *result)["corrected"] for result in results]
            for key in ("std_uT", "radius_rmse_uT"):
                self.assertLess(abs(before[key]-after[key]), 1e-6)
        self.assertLess(counts[1], counts[0] / 2)
        print(f"Recorded-data residual evaluations: finite difference {counts[0]}, analytic {counts[1]}")

    def test_noisy_data(self):
        rng = np.random.default_rng(8310)
        train = self.measured(sphere_points(1500)) + rng.normal(0, .05, (1500, 3))
        bias, matrix, radius = mag_fit.fit(train)
        np.testing.assert_allclose(bias, self.bias, atol=.02)
        result = mag_fit.statistics(self.measured(sphere_points(900, .31)), bias, matrix, radius)
        self.assertLess(result["corrected"]["relative_std"], .001)

    def test_planar_and_stationary_rejected(self):
        angle = np.linspace(0, 2*np.pi, 400)
        plane = np.column_stack((40*np.cos(angle), 40*np.sin(angle), np.full(400, 12)))
        for samples in (plane, np.ones((50, 3))):
            with self.assertRaisesRegex(ValueError, "退化"):
                mag_fit.fit(samples)

    def test_solver_failure_rejected(self):
        samples = self.measured(sphere_points(50))
        with patch.object(mag_fit, "least_squares", return_value=SimpleNamespace(success=False)):
            with self.assertRaisesRegex(ValueError, "未收敛"):
                mag_fit.fit(samples)

    def test_csv_and_failed_cli_do_not_write_parameters(self):
        with tempfile.TemporaryDirectory() as folder:
            path, output = Path(folder)/"input.csv", Path(folder)/"fit.json"
            samples = np.ones((30, 3))
            np.savetxt(path, samples, delimiter=",", header="mx_uT,my_uT,mz_uT", comments="")
            np.testing.assert_array_equal(mag_fit.read_samples(path), samples)
            with patch("sys.argv", [str(SCRIPT), str(path), "-o", str(output)]), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    mag_fit.main()
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
