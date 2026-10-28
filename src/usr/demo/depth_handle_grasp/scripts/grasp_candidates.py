"""Sample the existing fitted upper-face centreline; TCP X rod, Y closure, Z approach."""
from dataclasses import dataclass
import numpy as np
from grasp_geometry import validate, transform


def unit(v):
    v = np.asarray(v, float)
    if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-9:
        raise ValueError('invalid direction')
    return v / np.linalg.norm(v)


@dataclass
class Candidate:
    id: int
    tcp: np.ndarray
    arc_m: float
    end_clearance_m: float
    width_m: float
    thickness_m: float
    normal: np.ndarray
    tangent: np.ndarray
    rmse_m: float
    metadata: dict

    def to_dict(self):
        return dict(id=self.id, tcp=self.tcp.tolist(), arc_m=self.arc_m,
                    end_clearance_m=self.end_clearance_m, width_m=self.width_m,
                    thickness_m=self.thickness_m, normal=self.normal.tolist(),
                    tangent=self.tangent.tolist(), rmse_m=self.rmse_m, metadata=self.metadata)


def generate(result, base_depth, visual, cfg, pad_bounds):
    validate(base_depth)
    if not result.get('success') or not result.get('pose'):
        raise ValueError('visual pose unavailable')
    if result.get('coordinate_frame') != 'depth_optical' or result.get('units') != 'metres':
        raise ValueError('unsupported measurement frame/units')
    m = result['measurement']
    if (not np.isfinite([m['plane_rmse_m'], m['plane_inlier_ratio']]).all()
            or m['plane_rmse_m'] < 0 or not 0 <= m['plane_inlier_ratio'] <= 1
            or m['plane_rmse_m'] > cfg['max_rmse_m']
            or m['plane_inlier_ratio'] < cfg['min_inlier_ratio']):
        raise ValueError('fit quality below configured threshold')
    thickness = float(visual['thickness_m'])
    width = float(m['width_m'])
    if not np.isfinite([width, thickness]).all() or min(width, thickness) <= 0:
        raise ValueError('invalid section dimensions')
    if 'surface_centerline_m' in m:
        curve = np.asarray(m['surface_centerline_m'], float)
        normals = np.asarray(m['centerline_outward_normals'], float)
    else:
        if m.get('fit_model') or m.get('bow_sag_m', 0) > 0:
            raise ValueError('curved fit lacks exported centreline; rerun existing detector')
        axis = unit(m['axis_direction'])
        curve = np.asarray(m['surface_center_m']) + np.array([-.5, .5])[:, None]*m['length_m']*axis
        normals = np.tile(unit(m['outward_normal']), (2, 1))
    if curve.ndim != 2 or curve.shape[1] != 3 or len(curve) < 2 or normals.shape != curve.shape:
        raise ValueError('invalid fitted curve/normal arrays')
    if not np.isfinite(curve).all() or not np.isfinite(normals).all():
        raise ValueError('nonfinite fitted curve')
    segments = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    if np.any(segments < 1e-9):
        raise ValueError('degenerate fitted curve')
    arc = np.r_[0., np.cumsum(segments)]
    bounds = np.asarray(pad_bounds, float)
    if bounds.shape != (2, 2, 3) or not np.isfinite(bounds).all():
        raise ValueError('missing CAD pad bounds')
    half_pad = float(np.max(abs(bounds[:, :, 0])))
    margin = cfg['end_margin_m'] + half_pad + cfg['min_contact_margin_m']
    step = cfg['sample_spacing_m']; limit = cfg['max_candidates']
    if not np.isfinite([margin, step]).all() or margin < 0 or step <= 0 or limit < 1:
        raise ValueError('invalid sampling parameters')
    if arc[-1] < 2*margin:
        return [], [dict(stage='sampling', reason='rod too short for CAD pads and end margins')]
    positions = np.arange(margin, arc[-1]-margin+1e-10, step)
    positions = sorted(set(positions.tolist()+[arc[-1]/2]), key=lambda s: (abs(s-arc[-1]/2), s))
    out, rejected = [], []
    for s in positions:
        i = min(np.searchsorted(arc, s, side='right')-1, len(segments)-1)
        alpha = (s-arc[i])/segments[i]
        surface = (1-alpha)*curve[i]+alpha*curve[i+1]
        tangent = unit(base_depth[:3, :3] @ (curve[i+1]-curve[i]))
        normal = unit(base_depth[:3, :3] @ ((1-alpha)*normals[i]+alpha*normals[i+1]))
        center = base_depth[:3, :3] @ (surface-thickness*normal_camera(normals, i, alpha)/2)+base_depth[:3, 3]
        for angle in cfg['approach_angles_deg']:
            for sign in cfg['orientation_signs']:
                try:
                    if sign not in (-1, 1) or not np.isfinite(angle) or abs(angle) > cfg['max_face_angle_deg']:
                        raise ValueError('orientation outside configured allowed range')
                    z = unit(-normal*np.cos(np.radians(angle)) + np.cross(normal, tangent)*np.sin(np.radians(angle)))
                    if z[2] > -np.cos(np.radians(cfg['max_downward_angle_deg'])):
                        raise ValueError('approach not sufficiently downward in base frame')
                    x = sign*unit(tangent-z*np.dot(tangent, z))
                    if abs(np.dot(np.cross(z, x), tangent)) > 1e-6:
                        raise ValueError('closure axis incompatible with rod')
                    tcp = transform(center, np.column_stack((x, np.cross(z, x), z)))
                    out.append(Candidate(len(out), tcp, float(s), float(min(s, arc[-1]-s)-half_pad),
                                         width, thickness, normal, tangent, m['plane_rmse_m'],
                                         dict(contact_source='inferred_rectangular_section',
                                              dimension_source=visual.get('dimension_source', 'configured'),
                                              timestamp_ms=result.get('timestamp_ms'),
                                              frame=result['coordinate_frame'], approach_angle_deg=angle)))
                except ValueError as e:
                    rejected.append(dict(arc_m=float(s), stage='orientation', reason=str(e)))
                if len(out) >= limit:
                    return out, rejected
    return out, rejected


def normal_camera(normals, i, alpha):
    return unit((1-alpha)*normals[i]+alpha*normals[i+1])
