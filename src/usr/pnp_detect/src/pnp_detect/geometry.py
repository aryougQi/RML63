"""Image-space ordering does NOT establish a physical basket orientation."""
import cv2
import numpy as np

ORDER = ["image_top_left", "image_top_right", "image_bottom_right", "image_bottom_left"]
ORDER_RULE = ("Convex cyclic order, clockwise in image coordinates (x right, y down); "
              "start at minimum x+y, ties by y then x. Labels are image-relative only.")


def order_corners(points):
    p = np.asarray(points, dtype=np.float64)
    if p.shape != (4, 2) or not np.isfinite(p).all():
        raise ValueError("需要四个有限的二维坐标，形状必须为 (4,2)")
    if len(np.unique(p, axis=0)) != 4:
        raise ValueError("角点存在重复")
    hull = cv2.convexHull(p.astype(np.float32)).reshape(-1, 2)
    if len(hull) != 4:
        raise ValueError("四点不能构成凸四边形")
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1]-center[1], p[:, 0]-center[0]))]
    start = min(range(4), key=lambda i: (p[i].sum(), p[i, 1], p[i, 0]))
    return np.roll(p, -start, axis=0)


def validate_quad(points, width, height, *, min_area_ratio=0.015,
                  max_area_ratio=0.85, min_side_px=25, min_angle_deg=25,
                  max_angle_deg=155, border_margin_px=3, **_):
    # 输入必须已经是循环顺序；自交输入在这里拒绝，不自动修复。
    p = np.asarray(points, dtype=np.float64)
    if p.shape != (4, 2) or not np.isfinite(p).all():
        return False, {"reason": "non_finite_or_invalid_shape"}
    if not cv2.isContourConvex(p.astype(np.float32)):
        return False, {"reason": "non_convex_duplicate_or_self_intersecting"}
    if ((p[:, 0] < border_margin_px).any() or (p[:, 0] >= width-border_margin_px).any()
            or (p[:, 1] < border_margin_px).any() or (p[:, 1] >= height-border_margin_px).any()):
        return False, {"reason": "outside_image_or_touches_border"}
    sides = np.linalg.norm(np.roll(p, -1, axis=0)-p, axis=1)
    area_ratio = abs(cv2.contourArea(p.astype(np.float32))) / (width*height)
    angles = []
    for i in range(4):
        a, b = p[(i-1)%4]-p[i], p[(i+1)%4]-p[i]
        angles.append(float(np.degrees(np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)), -1, 1)))))
    metrics = {"area_ratio": float(area_ratio), "side_lengths_px": sides.tolist(), "angles_deg": angles}
    reason = None
    if sides.min() < min_side_px: reason = "side_too_short"
    elif not min_area_ratio <= area_ratio <= max_area_ratio: reason = "area_out_of_range"
    elif min(angles) < min_angle_deg or max(angles) > max_angle_deg: reason = "degenerate_angles"
    return reason is None, dict(metrics, reason=reason)


def line_intersection(line_a, line_b):
    """Each line is (point, direction); reject near-parallel lines."""
    a, u = map(lambda x: np.asarray(x, dtype=float), line_a)
    b, v = map(lambda x: np.asarray(x, dtype=float), line_b)
    mat = np.column_stack((u, -v))
    if abs(np.linalg.det(mat)) < 1e-6:
        raise ValueError("parallel_lines")
    return a + np.linalg.solve(mat, b-a)[0]*u
