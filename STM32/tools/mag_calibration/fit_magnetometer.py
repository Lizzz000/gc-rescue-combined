"""用独立三轴磁场 CSV 拟合硬铁偏移与对称软铁矩阵，单位为芯片坐标 µT。

模型参考 PX4 lm_mag_fit / lm_ellipsoid_fit_iteration（BSD-3-Clause）：
https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/commander/lm_fit.cpp
Copyright (c) 2021 PX4 Development Team. 许可见 LICENSES/PX4-BSD-3-Clause.txt。
沿用先球拟合固定 R，再最小化 R-||A(m-b)|| 的模型；SciPy 求解与统计为本地适配，
不移植 PX4 的迭代求解器。A 为对称矩阵（6参数），b 为偏移（3参数）。
半径由球拟合确定，不能当作已知地磁强度；模长拟合也不标定IMU安装旋转或地理北。
依赖：numpy、scipy。用法：python fit_magnetometer.py train.csv -o fit.json --validate check.csv
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def read_samples(path):
    """读取三列原始 µT；允许一行非数字表头，不静默丢弃无效样本。"""
    with Path(path).open(encoding="utf-8-sig") as stream:
        first = stream.readline().strip().split(",")
    try:
        [float(value) for value in first]
        skip = 0
    except ValueError:
        skip = 1
    samples = np.loadtxt(path, delimiter=",", skiprows=skip, ndmin=2, encoding="utf-8-sig")
    if samples.shape[1] != 3 or not np.all(np.isfinite(samples)):
        raise ValueError("CSV必须是有限数值的三列芯片坐标磁场，单位µT")
    return samples


def matrix_from_parameters(p):
    """按PX4的diag[XYZ]、offdiag[XY,XZ,YZ]顺序构造对称矩阵。"""
    xx, yy, zz, xy, xz, yz = p
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def apply_calibration(samples, offset, matrix):
    """每行芯片样本只校正一次：m_corrected = A @ (m_raw - b)。"""
    return (np.asarray(samples) - offset) @ np.asarray(matrix).T


def fit(samples):
    """先球后椭球拟合；退化、未收敛或非正定结果直接报错，不生成系数。"""
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) < 10 or not np.isfinite(samples).all():
        raise ValueError("至少需要10个有限三轴样本，且应覆盖三维方向")
    origin = samples.mean(axis=0)
    scale = np.sqrt(np.mean(np.sum((samples - origin)**2, axis=1)))
    if scale == 0:
        raise ValueError("样本退化：所有磁场读数相同")
    points = (samples - origin) / scale
    x, y, z = points.T
    design = np.column_stack((x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones(len(x))))
    if np.linalg.matrix_rank(design) < 9:
        raise ValueError("样本退化：无法辨识三维椭球，请补充翻转方向")
    sphere = least_squares(lambda p: p[0] - np.linalg.norm(points - p[1:], axis=1),
                           [1., 0., 0., 0.], x_scale="jac", max_nfev=1000,
                           ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if not sphere.success or not np.isfinite(sphere.x).all() or sphere.x[0] <= 0:
        raise ValueError("球拟合未收敛到有效正半径")
    radius = sphere.x[0]
    initial = np.r_[sphere.x[1:], [1., 1., 1., 0., 0., 0.]]
    ellipsoid = least_squares(
        lambda p: radius - np.linalg.norm(apply_calibration(points, p[:3], matrix_from_parameters(p[3:])), axis=1),
        initial, x_scale="jac", max_nfev=1000, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if not ellipsoid.success or not np.isfinite(ellipsoid.x).all():
        raise ValueError("椭球拟合未收敛")
    matrix = matrix_from_parameters(ellipsoid.x[3:])
    if np.linalg.matrix_rank(ellipsoid.jac) < 9 or np.min(np.linalg.eigvalsh(matrix)) <= 0:
        raise ValueError("拟合退化或校准矩阵不是正定矩阵")
    return origin + scale * ellipsoid.x[:3], matrix, radius * scale


def statistics(samples, offset, matrix, radius):
    """报告模长误差及方向覆盖，不用任意数据阈值自动宣布标定成功。"""
    corrected = apply_calibration(samples, offset, matrix)
    result = {"samples": len(samples)}
    for name, vectors in (("raw", samples), ("corrected", corrected)):
        lengths = np.linalg.norm(vectors, axis=1)
        result[name] = {"mean_uT": float(lengths.mean()), "std_uT": float(lengths.std()),
                        "p2p_uT": float(np.ptp(lengths)), "relative_std": float(lengths.std()/lengths.mean()),
                        "radius_rmse_uT": float(np.sqrt(np.mean((lengths-radius)**2)))}
    lengths = np.linalg.norm(corrected, axis=1)
    directions = corrected[lengths > 0] / lengths[lengths > 0, None]
    octants = np.sum((directions >= 0) * [1, 2, 4], axis=1)
    result["coverage"] = {"octant_counts_xyz_bits": np.bincount(octants, minlength=8).tolist(),
                          "direction_min_xyz": directions.min(axis=0).tolist(),
                          "direction_max_xyz": directions.max(axis=0).tolist(),
                          "direction_second_moment_eigenvalues": np.linalg.eigvalsh(directions.T @ directions / len(directions)).tolist()}
    return result


def main():
    """先完成拟合及独立验证统计，全部有效后才写JSON参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--validate", type=Path, help="另一轮独立翻转采样CSV；不参与拟合")
    args = parser.parse_args()
    try:
        samples = read_samples(args.csv)
        offset, matrix, radius = fit(samples)
        result = {"model": "corrected_uT = matrix @ (raw_uT - offset_uT)",
                  "source_revision": "PX4 30e763b6780061d70a14894e3e8b06e6a656f9b8",
                  "offset_uT": offset.tolist(), "matrix": matrix.tolist(), "radius_uT": float(radius),
                  "training": statistics(samples, offset, matrix, radius)}
        if args.validate:
            result["validation"] = statistics(read_samples(args.validate), offset, matrix, radius)
    except (ValueError, OSError, np.linalg.LinAlgError) as error:
        parser.error(str(error))
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    args.output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
