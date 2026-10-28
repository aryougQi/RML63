import cv2
import numpy as np
import yaml


def load_config(path):
    try:
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise ValueError(f"YAML 配置格式错误: {exc}") from exc
    if not isinstance(cfg, dict) or not all(isinstance(cfg.get(k), dict) for k in ("preprocessing", "detection", "input_geometry")):
        raise ValueError("配置必须包含 preprocessing/detection/input_geometry 映射")
    p, d = cfg["preprocessing"], cfg["detection"]
    if p["blur_kernel"] < 1 or p["blur_kernel"] % 2 != 1 or p["close_kernel"] < 1:
        raise ValueError("blur_kernel 必须为正奇数，close_kernel 必须为正数")
    if not 0 <= p["canny_low"] < p["canny_high"] <= 255:
        raise ValueError("Canny 阈值应满足 0 <= low < high <= 255")
    for key in ("min_edge_support", "min_side_support", "min_contour_coverage", "min_line_support", "max_unsupported_run_ratio"):
        if not 0 <= d[key] <= 1:
            raise ValueError(f"{key} 必须在 [0,1] 内")
    if not 0 < d["min_area_ratio"] < d["max_area_ratio"] <= 1:
        raise ValueError("面积比例范围无效")
    if not 0 < d["min_angle_deg"] < d["max_angle_deg"] < 180:
        raise ValueError("角度范围无效")
    for key in ("min_side_px", "edge_distance_px", "hough_threshold", "hough_min_line_length",
                "line_distance_px", "refine_distance_px", "max_refine_shift_px", "duplicate_distance_px", "max_candidates"):
        if d[key] <= 0: raise ValueError(f"{key} 必须为正")
    if not d["approx_epsilon_ratios"] or any(not 0 < x < 0.1 for x in d["approx_epsilon_ratios"]):
        raise ValueError("轮廓近似 epsilon 比例必须在 (0,0.1) 内")
    if (p["hsv_lower"] is None) != (p["hsv_upper"] is None):
        raise ValueError("HSV 上下界必须同时配置")
    if p["hsv_lower"] is not None:
        lo, hi = np.asarray(p["hsv_lower"]), np.asarray(p["hsv_upper"])
        if lo.shape != (3,) or hi.shape != (3,) or (lo < 0).any() or (hi > [179,255,255]).any() or (lo > hi).any():
            raise ValueError("HSV 范围无效")
    for key in ("max_endpoint_gap_px", "max_unsupported_gap_px"):
        if key in d and d[key] <= 0: raise ValueError(f"{key} 必须为正")
    if not 0 < d.get("max_near_parallel_pair_angle_deg",90) <= 90:
        raise ValueError("max_near_parallel_pair_angle_deg 必须在 (0,90] 内")
    lines=d.get("line_proposals",{})
    if not isinstance(lines,dict): raise ValueError("line_proposals 必须为映射")
    if not 4 <= lines.get("max_lines",24) <= 40: raise ValueError("max_lines 必须在 [4,40] 内")
    if not 0 < lines.get("max_opposite_angle_deg",28) < 60: raise ValueError("对边夹角范围无效")
    polarity=d.get("rim_polarity",{})
    if not isinstance(polarity,dict): raise ValueError("rim_polarity 必须为映射")
    if not 0 <= polarity.get("normal_search_px",0) <= d["edge_distance_px"]:
        raise ValueError("normal_search_px 不得超过真实边缘距离容差")
    if not 0 < polarity.get("sample_offset_px",4) <= 20 or not 0 <= polarity.get("min_side_contrast",8) <= 255:
        raise ValueError("边框明暗采样距离或阈值无效")
    region=d.get("region_proposals",{})
    if not isinstance(region,dict): raise ValueError("region_proposals 必须为映射")
    if region.get("enabled",False):
        lo=np.asarray(region.get("hsv_lower"),dtype=float);hi=np.asarray(region.get("hsv_upper"),dtype=float)
        if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite(lo).all() or not np.isfinite(hi).all() or (lo<0).any() or (hi>[179,255,255]).any() or (lo>hi).any():
            raise ValueError("region_proposals HSV 范围无效")
        kernel=region.get("close_kernel",3)
        if type(kernel) is not int or kernel<1 or kernel>31 or kernel%2!=1: raise ValueError("区域闭运算核必须为 1..31 的奇数")
        if not 0<region.get("fit_band_ratio",.08)<=.15: raise ValueError("区域直线拟合带宽比例无效")
    geom = cfg["input_geometry"]
    if geom["status"] not in ("unknown", "native", "transformed"):
        raise ValueError("input_geometry.status 应为 unknown/native/transformed")
    if geom["status"] != "unknown":
        size = geom["source_resolution_wh"]
        matrix = np.asarray(geom["source_to_input_3x3"], dtype=float)
        if not isinstance(size, list) or len(size) != 2 or min(size) <= 0 or matrix.shape != (3,3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-9:
            raise ValueError("已知图像坐标关系时必须提供源分辨率和可逆的 3x3 变换")
    return cfg


def load_image(path):
    from pathlib import Path
    path = Path(path)
    if not path.is_file(): raise ValueError(f"图像不存在: {path}")
    raw = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED) if raw.size else None
    if img is None: raise ValueError("无法解码图像：空文件、格式不支持或文件已损坏")
    metadata = {"width": int(img.shape[1]), "height": int(img.shape[0]),
                "channels": 1 if img.ndim == 2 else int(img.shape[2]),
                "dtype": str(img.dtype), "file_suffix": path.suffix.lower(),
                "decoded_format": "uint8 grayscale/BGR/BGRA (OpenCV decoder)",
                "encoded_format": "png" if raw[:8].tobytes() == b"\x89PNG\r\n\x1a\n" else "jpeg" if raw[:2].tobytes() == b"\xff\xd8" else "other_opencv_supported"}
    if img.dtype != np.uint8: raise ValueError(f"只接受 8-bit RGB 截图，实际 dtype={img.dtype}")
    if min(img.shape[:2]) < 32: raise ValueError("图像尺寸太小，宽高至少 32 像素")
    if img.ndim == 2: img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:
        if (img[:,:,3] != 255).any(): raise ValueError("存在透明像素，请先明确背景并导出不透明 RGB 图像")
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif img.shape[2] != 3: raise ValueError("不支持的通道数")
    return img, metadata


def preprocess(image, cfg):
    p = cfg["preprocessing"]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (p["blur_kernel"],)*2, 0)
    edges = cv2.Canny(blurred, p["canny_low"], p["canny_high"])
    mask = np.full(gray.shape, 255, np.uint8)
    if p["hsv_lower"] is not None:
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array(p["hsv_lower"]), np.array(p["hsv_upper"]))
        # 颜色只作为搜索先验；保留其边界附近的真实灰度边缘。
        mask = cv2.dilate(mask, np.ones((5,5), np.uint8))
    if p["roi"] is not None:
        x,y,w,h = p["roi"]
        if any(not isinstance(v, int) for v in (x,y,w,h)) or x<0 or y<0 or w<=0 or h<=0 or x+w>image.shape[1] or y+h>image.shape[0]:
            raise ValueError("ROI 必须是输入图像范围内的整数 [x,y,w,h]")
        roi = np.zeros_like(mask); roi[y:y+h,x:x+w] = 255
        mask = cv2.bitwise_and(mask, roi)
    edges = cv2.bitwise_and(edges, mask)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((p["close_kernel"],)*2, np.uint8))
    return {"gray": gray, "blurred": blurred, "search_mask": mask, "edges": edges, "closed_edges": closed}


def scale_detection_parameters(config, width, height):
    """Scale length thresholds, never image pixels or camera coordinates."""
    import copy
    reference=config['detection'].get('reference_long_side_px')
    if reference is None:return config,1.
    if not np.isfinite(reference) or reference<=0:raise ValueError('reference_long_side_px must be positive')
    scale=max(width,height)/float(reference)
    effective=copy.deepcopy(config);d=effective['detection']
    for key in ('min_side_px','border_margin_px','edge_distance_px','line_distance_px',
                'refine_distance_px','max_refine_shift_px','duplicate_distance_px',
                'max_endpoint_gap_px','max_unsupported_gap_px'):
        if key in d:d[key]=d[key]*scale
    for key in ('hough_threshold','hough_min_line_length','hough_max_line_gap'):
        if key in d:d[key]=max(1,int(round(d[key]*scale)))
    for group,keys in [('rim_polarity',('sample_offset_px','normal_search_px')),
                       ('line_proposals',('merge_distance_px',))]:
        for key in keys:
            if key in d.get(group,{}):d[group][key]*=scale
    return effective,scale
