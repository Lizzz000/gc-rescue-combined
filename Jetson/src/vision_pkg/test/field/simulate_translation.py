"""离线模拟相机沿地面八向平移，验证紫框和红底同时出画时的鸟瞰配准。"""

import argparse
from pathlib import Path

import cv2
import numpy as np


WIDTH, DEPTH, INSET, DIVIDER = 0.66, 0.36, 0.03, 0.02
HEIGHT = 0.02
SIZE = (1280, 720)
PX_PER_M = 400
SEARCH = 240
PADDING = 12
GROUND = (222, 224, 226)
PURPLE = (190, 95, 120)
RED = (70, 72, 160)
BLACK = (28, 28, 30)
COLOURS = (GROUND, PURPLE, RED, BLACK)
INNER = np.float32([
    [INSET, DEPTH - INSET], [WIDTH - INSET, DEPTH - INSET],
    [WIDTH - INSET, INSET], [INSET, INSET],
])
CENTRES = np.float32([[0.18, 0.18], [0.48, 0.18]])
ORIGIN = SEARCH + PADDING
TILE = (round(WIDTH * PX_PER_M) + 2 * PADDING, round(DEPTH * PX_PER_M) + 2 * PADDING)
CANVAS = (TILE[0] + 2 * SEARCH, TILE[1] + 2 * SEARCH)
TO_GRID = np.array([[PX_PER_M, 0, ORIGIN], [0, -PX_PER_M, ORIGIN + DEPTH * PX_PER_M], [0, 0, 1]])


def ground_samples(left, right):
    xx, yy = np.meshgrid(np.linspace(left, right, 160), np.linspace(INSET, DEPTH - INSET, 160))
    return np.float32(np.column_stack((xx.ravel(), yy.ravel())))


AREA_SAMPLES = ground_samples(INSET, WIDTH - INSET)
RED_LEFT = ground_samples(INSET, (WIDTH - DIVIDER) / 2)
RED_RIGHT = ground_samples((WIDTH + DIVIDER) / 2, WIDTH - INSET)
GROUND_CM2 = 10000 * (WIDTH - 2 * INSET) * (DEPTH - 2 * INSET)
HALF_RED_CM2 = 5000 * (WIDTH - 2 * INSET - DIVIDER) * (DEPTH - 2 * INSET)


def project(points, transform):
    return cv2.perspectiveTransform(np.float32(points)[None], transform)[0]


def camera(shift, yaw=0.0, height=0.64):
    eye = np.array([WIDTH / 2, -0.27, height])
    forward = np.array([WIDTH / 2, DEPTH / 2, 0.0]) - eye
    forward /= np.linalg.norm(forward)
    sine, cosine = np.sin(yaw), np.cos(yaw)
    forward[:2] = (cosine * forward[0] - sine * forward[1], sine * forward[0] + cosine * forward[1])
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    axes = np.stack((right, np.cross(forward, right), forward))
    eye[:2] += shift
    focal = 0.62 * SIZE[0]
    intrinsic = np.array([[focal, 0, SIZE[0] / 2], [0, focal, SIZE[1] / 2], [0, 0, 1]])
    projection = intrinsic @ np.column_stack((axes, -axes @ eye))
    return projection[:, [0, 1, 3]], projection


def render(projection, height, divider=True, size=SIZE):
    image = np.full((size[1], size[0], 3), GROUND, np.uint8)

    def paint(xy, height, colour):
        world = np.column_stack((xy, np.broadcast_to(height, (len(xy),)), np.ones(len(xy))))
        pixels = world @ projection.T
        if (pixels[:, 2] <= 0).any():
            return
        pixels = np.rint(16 * pixels[:, :2] / pixels[:, 2:]).astype(np.int32)
        cv2.fillConvexPoly(image, pixels, colour, cv2.LINE_AA, 4)

    def rectangle(x0, y0, x1, y1, colour, height=0):
        paint([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], height, colour)

    rectangle(0, 0, WIDTH, DEPTH, PURPLE)
    rectangle(INSET, INSET, WIDTH - INSET, DEPTH - INSET, RED)
    if divider:
        left, right = (WIDTH - DIVIDER) / 2, (WIDTH + DIVIDER) / 2
        rectangle(left, INSET, right, DEPTH - INSET, BLACK)
        if height:
            paint([(left, INSET), (right, INSET), (right, INSET), (left, INSET)],
                  [0, 0, height, height], BLACK)
            for side in (left, right):
                paint([(side, INSET), (side, DEPTH - INSET), (side, DEPTH - INSET), (side, INSET)],
                      [0, 0, height, height], BLACK)
            rectangle(left, INSET, right, DEPTH - INSET, BLACK, height)
    if height:
        rectangle(0, DEPTH - INSET, WIDTH, DEPTH, PURPLE, height)
        rectangle(0, INSET, INSET, DEPTH - INSET, PURPLE, height)
        rectangle(WIDTH - INSET, INSET, WIDTH, DEPTH - INSET, PURPLE, height)
        rectangle(0, 0, WIDTH, INSET, PURPLE, height)
        paint([(0, 0), (WIDTH, 0), (WIDTH, 0), (0, 0)], [0, 0, height, height], PURPLE)
    return image


def visible_points(points):
    return ((points >= 0) & (points < np.array(SIZE))).all(axis=1)


def missing_fraction(homography, samples=AREA_SAMPLES):
    return 1.0 - np.mean(visible_points(project(samples, homography)))


def rectified(image, ground_to_image):
    transform = TO_GRID @ np.linalg.inv(ground_to_image)
    view = cv2.warpPerspective(image, transform, CANVAS)
    mask = cv2.warpPerspective(np.ones(image.shape[:2], np.uint8), transform, CANVAS,
                               flags=cv2.INTER_NEAREST)
    return view, mask.astype(np.float32)


def labels(image):
    # 合成图使用固定颜色。工程接入时应换成识别程序已有的 Lab 掩膜。
    pixels = image.astype(np.float32)
    distances = [np.sum((pixels - colour) ** 2, axis=2) for colour in COLOURS]
    return np.argmin(distances, axis=0).astype(np.uint8)


def class_views(image, ground_h, raised_h):
    ground_view, ground_visible = rectified(image, ground_h)
    ground_labels = labels(ground_view)
    if raised_h is None:
        return [(ground_labels == colour, ground_visible) for colour in range(len(COLOURS))]
    # 紫围栏和黑带顶面与红色地面各用对应高度的单应矩阵。
    raised_view, raised_visible = rectified(image, raised_h)
    raised_labels = labels(raised_view)
    return [
        (ground_labels == 0, ground_visible),
        (raised_labels == 1, raised_visible),
        (ground_labels == 2, ground_visible),
        (raised_labels == 3, raised_visible),
    ]


def moved_ground(reference_h, x, y):
    offset = np.array([[1, 0, (x - SEARCH) / PX_PER_M],
                       [0, 1, -(y - SEARCH) / PX_PER_M], [0, 0, 1]])
    return reference_h @ offset


def refine_pose(scores, current, reference_h, reference_camera, height):
    # 两个高度的配准提供初值。对少数候选按实际相机模型重新投影，复核颜色和遮挡关系。
    size = (SIZE[0] // 4, SIZE[1] // 4)
    observed = labels(cv2.resize(current, size, interpolation=cv2.INTER_AREA))
    observed_masks = [observed == colour for colour in (1, 2, 3)]
    scale = np.diag([0.25, 0.25, 1.0])
    projection = reference_camera.copy()
    evaluated = {}

    def evaluate_window(center_x, center_y, radius, step):
        for y in range(max(0, center_y - radius), min(scores.shape[0] - 1, center_y + radius) + 1, step):
            for x in range(max(0, center_x - radius), min(scores.shape[1] - 1, center_x + radius) + 1, step):
                if (x, y) in evaluated or scores[y, x] < 0:
                    continue
                projection[:, [0, 1, 3]] = moved_ground(reference_h, x, y)
                model = labels(render(scale @ projection, height, size=size))
                overlap = []
                for colour, observed_mask in zip((1, 2, 3), observed_masks):
                    model_mask = model == colour
                    union = np.count_nonzero(observed_mask | model_mask)
                    overlap.append(np.count_nonzero(observed_mask & model_mask) / max(union, 1))
                evaluated[x, y] = sum(overlap) / len(overlap)

    peaks = scores.copy()
    for _ in range(3):
        _, peak, _, (x, y) = cv2.minMaxLoc(peaks)
        if peak < 0.55:
            break
        evaluate_window(x, y, 12, 4)
        peaks[max(0, y - 8):y + 9, max(0, x - 8):x + 9] = -1
    if not evaluated:
        return None
    x, y = max(evaluated, key=evaluated.get)
    evaluate_window(x, y, 5, 1)
    x, y = max(evaluated, key=evaluated.get)
    score = evaluated[x, y]
    alternatives = [quality for (a, b), quality in evaluated.items() if max(abs(a - x), abs(b - y)) > 4]
    if score < 0.82 or (alternatives and score - max(alternatives) < 0.025):
        return None
    return x, y


def recover(reference_classes, current_image, reference_h, reference_camera, raised_h, height):
    current_classes = class_views(current_image, reference_h, raised_h)
    support = np.zeros((2 * SEARCH + 1, 2 * SEARCH + 1), np.float32)
    matched = np.zeros_like(support)
    total_weight = 0.0
    for reference, (current, visible) in zip(reference_classes, current_classes):
        reference = reference.astype(np.uint8)
        # 只用各颜色的内侧边沿，避免红底纯色面积淹没边界位置证据。
        edges = reference - cv2.erode(reference, np.ones((5, 5), np.uint8))
        reference_class = edges.astype(np.float32)
        current_class = current.astype(np.float32) * visible
        total_weight += float(edges.sum())
        support += cv2.matchTemplate(visible, reference_class, cv2.TM_CCORR)
        matched += cv2.matchTemplate(current_class, reference_class, cv2.TM_CCORR)
    scores = matched / np.maximum(support, 1.0)
    scores[support < 0.55 * total_weight] = -1
    _, peak, _, (x, y) = cv2.minMaxLoc(scores)
    if height:
        refined = refine_pose(scores, current_image, reference_h, reference_camera, height)
        if refined is None:
            return None
        x, y = refined
    else:
        # 任一轴还能明显移动却几乎同样吻合，说明这一帧没有足够的二维约束。
        scores[max(0, y - 4):y + 5, max(0, x - 4):x + 5] = -1
        if peak < 0.80 or peak - float(scores.max()) < 0.025:
            return None

    centre = x + PADDING + round(WIDTH * PX_PER_M / 2)
    y0 = y + PADDING + round(INSET * PX_PER_M)
    y1 = y + PADDING + round((DEPTH - INSET) * PX_PER_M)
    current_black, visible = current_classes[3]
    black = current_black[y0:y1, centre - 4:centre + 5]
    band_in_view = visible[y0:y1, centre - 4:centre + 5]
    supported = (black * band_in_view).sum(axis=1) >= 3
    if supported.sum() < 0.35 * (y1 - y0):
        return None

    return moved_ground(reference_h, x, y)


def run(output, height, target_missing):
    reference_h, reference_camera = camera((0, 0))
    reference_image = render(reference_camera, height)
    raised_h = None
    if height:
        raised_h = reference_h.copy()
        raised_h[:, 2] += height * reference_camera[:, 2]
    reference_classes = [
        mask[SEARCH:SEARCH + TILE[1], SEARCH:SEARCH + TILE[0]]
        for mask, _ in class_views(reference_image, reference_h, raised_h)
    ]
    if output:
        output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output / "reference.png"), reference_image)
    directions = {
        "left": (-1, 0), "right": (1, 0), "up": (0, 1), "down": (0, -1),
        "up-left": (-1, 1), "up-right": (1, 1),
        "down-left": (-1, -1), "down-right": (1, -1),
    }
    print("direction     missing% true_cm2 estimated_cm2 center_error_px left_red_cm2 right_red_cm2")
    for name, direction in directions.items():
        for scale in np.arange(0.0, SEARCH / PX_PER_M, 0.002):
            actual_h, actual_camera = camera(np.array(direction) * scale)
            missing = missing_fraction(actual_h)
            if missing >= target_missing:
                break
        if missing < target_missing:
            print(f"{name:12} did not reach target crop")
            continue
        current = render(actual_camera, height)
        if output:
            cv2.imwrite(str(output / f"{name}.png"), current)
        result = recover(reference_classes, current, reference_h, reference_camera, raised_h, height)
        estimated = "rejected"
        if result is not None:
            actual = project(CENTRES, actual_h)
            recovered = project(CENTRES, result)
            error = np.linalg.norm(recovered - actual, axis=1).max()
            area = GROUND_CM2 * missing_fraction(result)
            left = HALF_RED_CM2 * missing_fraction(result, RED_LEFT)
            right = HALF_RED_CM2 * missing_fraction(result, RED_RIGHT)
            estimated = f"{area:10.1f} {error:14.2f} {left:12.1f} {right:13.1f}"
        print(f"{name:12} {100 * missing:8.2f} {GROUND_CM2 * missing:7.1f}    {estimated}")

    for name, image in (("blank", np.full_like(reference_image, GROUND)),
                        ("no-divider", render(reference_camera, height, divider=False))):
        accepted = recover(reference_classes, image, reference_h, reference_camera, raised_h, height) is not None
        print(f"{name:12} {'accepted' if accepted else 'rejected'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="写出可给 C++ 识别器回放的裁切样本")
    parser.add_argument("--planar", action="store_true", help="将围栏和黑带都放在地面上作对照")
    parser.add_argument("--missing-fraction", type=float, default=0.08, help="目标画外地面比例")
    args = parser.parse_args()
    if not 0.0 < args.missing_fraction < 1.0:
        parser.error("残缺比例须在 0 和 1 之间")
    run(args.output, 0.0 if args.planar else HEIGHT, args.missing_fraction)
