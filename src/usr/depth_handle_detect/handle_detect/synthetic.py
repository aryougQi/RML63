"""Ray-sampled rectangular upper face with explicit ground truth, NOT real data."""
import json
from pathlib import Path
import numpy as np
from .data import save_frame, write_json


def render(seed=0, angle=0.35, second=False, empty=False):
    rng = np.random.default_rng(seed)
    k = dict(width=320, height=240, fx=400., fy=400., ppx=160., ppy=120.,
             distortion_model_id=0, coefficients=[0.]*5)
    v, u = np.indices((240, 320))
    xx, yy = (u-160)*0.8/400, (v-120)*0.8/400
    t = xx*np.cos(angle)+yy*np.sin(angle)
    s = -xx*np.sin(angle)+yy*np.cos(angle)
    rod = (abs(t) <= 0.12) & (abs(s) <= 0.012)
    if second:
        rod |= (abs(xx) < 0.12) & (abs(yy-0.10) < 0.012)
    if empty:
        rod[:] = False
    z = np.full((240, 320), 1.10)
    # Basket rim is connected, wider/larger, at a distinct depth.
    rim = ((abs(xx) < 0.25) & (abs(yy) < 0.18) & ((abs(xx) > 0.23) | (abs(yy) > 0.16)))
    z[rim] = 0.95
    z[rod] = 0.8
    z += rng.normal(0, 0.0006, z.shape)
    d = np.rint(z/0.00025).astype(np.uint16)
    d[rng.random(d.shape) < 0.003] = 0
    meta = dict(intrinsics=k, depth_scale_m_per_unit=0.00025, origin='synthetic',
                frame_number=seed, timestamp_ms=seed*1000/30, coordinate_frame='depth_optical')
    return d, meta


def generate(output, count, config_path):
    out = Path(output)
    for i in range(count):
        d, m = render(i)
        save_frame(out/('frame_%06d' % i), d, m)
    cfg = json.loads(Path(config_path).read_text())
    cfg.update(roi_xywh=[65, 65, 190, 110], roi_confirms_handle_identity=True,
               dimensions_verified=True, approach_camera=[0, 0, 1], approach_verified=True)
    write_json(out/'synthetic_config.json', cfg)
    write_json(out/'ground_truth.json', dict(origin='synthetic', center_m=[0, 0, 0.809],
               surface_center_m=[0, 0, 0.8], axis_direction=[float(np.cos(.35)), float(np.sin(.35)), 0.],
               length_m=.24, width_m=.024, thickness_m=.018))
