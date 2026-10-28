from pathlib import Path
import cv2
import numpy as np
from .data import write_json
from .paths import detail_directory


def project(p, k):
    if k.get('distortion_model_id', 0) and np.any(k.get('coefficients', [])):
        import pyrealsense2 as rs
        intr = rs.intrinsics()
        for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
            setattr(intr, key, k[key])
        intr.model = rs.distortion(k['distortion_model_id'])
        intr.coeffs = k['coefficients']
        return tuple(np.rint(rs.rs2_project_point_to_pixel(intr, list(p))).astype(int))
    return tuple(np.rint([p[0]/p[2]*k['fx']+k['ppx'], p[1]/p[2]*k['fy']+k['ppy']]).astype(int))


def save_outputs(output, depth, meta, result, mask, fit):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    details = detail_directory(output)
    details.mkdir(parents=True, exist_ok=True)
    z = depth.astype(float)*meta['depth_scale_m_per_unit']
    valid = depth > 0
    gray = np.zeros(depth.shape, np.uint8)
    if valid.any():
        lo, hi = np.percentile(z[valid], [2, 98])
        gray[valid] = np.clip((z[valid]-lo)/max(hi-lo, 0.001)*255, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    vis[~valid] = 0
    cv2.imwrite(str(details/'depth_visualization.png'), vis)
    cv2.imwrite(str(details/'handle_mask.png'), mask)
    overlay = vis.copy()
    overlay[mask > 0] = (0, 255, 0)
    vis = cv2.addWeighted(vis, 0.65, overlay, 0.35, 0)
    if fit:
        k = meta['intrinsics']
        c, axis = fit['surface_center'], fit['axis']
        cv2.circle(vis, project(c, k), 4, (0, 0, 255), -1)
        cv2.arrowedLine(vis, project(c-axis*fit['length']/2, k),
                       project(c+axis*fit['length']/2, k), (255, 255, 255), 2)
        if 'centerline' in fit:
            curve = np.array([project(p, k) for p in fit['centerline']], np.int32)
            cv2.polylines(vis, [curve], False, (255, 0, 255), 1)
        if result.get('pose'):
            approach = np.asarray(result['pose']['approach_camera'])
            cv2.arrowedLine(vis, project(c-approach*.045, k), project(c, k), (255, 255, 0), 2)
            cv2.putText(vis, 'approach %.0f deg' % result['pose']['tilt_about_handle_deg'],
                        (8, 38), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 0), 1)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cv2.drawContours(vis, contours, -1, (0, 255, 255), 1)
    label = 'OK' if result['success'] else ('DETECTED / POSE UNREADY' if result.get('detection_success')
                                            else 'FAIL: '+result['reason'][:45])
    cv2.putText(vis, label,
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0) if result['success'] else (0, 0, 255), 1)
    cv2.imwrite(str(output/'handle_detection.png'), vis)
    points = fit['points'] if fit else np.empty((0, 3))
    with (details/'handle_points.ply').open('w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n' % len(points))
        np.savetxt(f, points, fmt='%.6f')
    write_json(details/'handle_pose.json', result)
