import numpy as np
from scipy.spatial.transform import Rotation


class Reject(ValueError):
    pass


def unit(v):
    v = np.asarray(v, dtype=float)
    if v.shape != (3,) or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-9:
        raise Reject('invalid approach vector')
    return v / np.linalg.norm(v)


def points_in_handle_section(points, fit, cfg, pixel_size_m):
    """Classify native returns inside the fitted strip's known cross section.

    The fitted centreline supplies the surface. Width/thickness come from the
    configured target; a bounded pixel footprint and the existing surface
    residual account for boundary measurements. No points are generated.
    """
    from scipy.spatial import cKDTree
    if not cfg.get('dimensions_verified') or cfg['thickness_m'] <= 0:
        raise Reject('handle section dimensions unverified')
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise Reject('invalid measured section points')
    if not np.isfinite(pixel_size_m) or pixel_size_m <= 0:
        raise Reject('invalid native pixel footprint')
    curve=np.asarray(fit.get('centerline',[
        fit['surface_center']-fit['axis']*fit['length']/2,
        fit['surface_center']+fit['axis']*fit['length']/2]),float)
    if curve.ndim!=2 or curve.shape[1]!=3 or len(curve)<2 or not np.isfinite(curve).all():
        raise Reject('invalid measured handle centreline')
    tangents=np.gradient(curve,axis=0)
    tangents/=np.linalg.norm(tangents,axis=1)[:,None]
    normal=unit(fit['normal'])
    normals=normal-tangents*(tangents@normal)[:,None]
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    across=np.cross(normals,tangents)
    _,indices=cKDTree(curve).query(points)
    delta=points-curve[indices]
    lateral=np.einsum('ij,ij->i',delta,across[indices])
    height=np.einsum('ij,ij->i',delta,normals[indices])
    along=np.einsum('ij,ij->i',delta,tangents[indices])
    boundary=min(pixel_size_m,cfg['width_tolerance_m']/2)
    residual=cfg['plane_inlier_m']
    inside=(abs(lateral)<=cfg['width_m']/2+boundary)
    inside&=(height>=-cfg['thickness_m']-residual)&(height<=residual)
    inside&=~((indices==0)&(along < -boundary))
    inside&=~((indices==len(curve)-1)&(along > boundary))
    return inside


def fit_handle(points, cfg):
    if len(points) < cfg['min_points']:
        raise Reject('insufficient points')
    # RANSAC plane: a rectangular rod presents its upper planar face.
    rng = np.random.default_rng(17)
    sample = points if len(points) <= 1500 else points[rng.choice(len(points), 1500, replace=False)]
    best = np.zeros(len(sample), dtype=bool)
    for _ in range(60):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-10:
            continue
        n /= np.linalg.norm(n)
        good = np.abs((sample-a) @ n) < cfg['plane_inlier_m']
        if good.sum() > best.sum():
            best = good
    if best.mean() < cfg['min_plane_inlier_ratio']:
        raise Reject('plane inlier ratio too low')
    anchor = sample[best].mean(axis=0)
    _, _, vt = np.linalg.svd(sample[best]-anchor, full_matrices=False)
    normal = vt[-1]
    if np.dot(normal, anchor) > 0:
        normal = -normal  # outward, toward observing camera
    residual = np.abs((points-anchor) @ normal)
    inliers = residual < cfg['plane_inlier_m']
    if inliers.mean() < cfg['min_plane_inlier_ratio']:
        raise Reject('full cloud plane inlier ratio too low')
    p = points[inliers]
    anchor = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p-anchor, full_matrices=False)
    axis = vt[0]
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    across = unit(np.cross(normal, axis))
    t, s = (p-anchor) @ axis, (p-anchor) @ across
    lo, hi = np.percentile(t, [0.5, 99.5])
    wl, wh = np.percentile(s, [0.5, 99.5])
    length, width = float(hi-lo), float(wh-wl)
    rmse = float(np.sqrt(np.mean(((p-anchor) @ normal)**2)))
    if abs(length-cfg['length_m']) > cfg['length_tolerance_m']:
        raise Reject('length mismatch: %.4f m' % length)
    if abs(width-cfg['width_m']) > cfg['width_tolerance_m']:
        raise Reject('width mismatch: %.4f m' % width)
    if rmse > cfg['max_plane_rmse_m']:
        raise Reject('plane residual too large')
    # Require broad coverage along the whole rod, not disconnected fragments.
    bins = np.histogram(t, bins=12, range=(lo, hi))[0]
    if np.min(bins) < max(2, len(p)*0.015):
        raise Reject('occlusion or incomplete axial coverage')
    surface = anchor + axis*(lo+hi)/2 + across*(wl+wh)/2
    return dict(surface_center=surface, axis=axis, normal=normal, length=length,
                width=width, rmse=rmse, inlier_ratio=float(inliers.mean()), points=p)


def _make_pose_at_tilt(fit, cfg):
    if cfg['shape'] not in ('rectangular_top_face', 'curved_rectangular_top_face'):
        raise Reject('only verified rectangular top faces are supported; cylinder needs a cylinder model')
    if not cfg['dimensions_verified']:
        raise Reject('physical dimensions and rectangular section not verified')
    if cfg['thickness_m'] <= 0 or not np.isfinite(cfg['thickness_m']):
        raise Reject('invalid physical thickness')
    if not cfg['approach_verified'] or cfg['approach_camera'] is None:
        raise Reject('vertical approach in camera frame not calibrated')
    vertical = unit(cfg['approach_camera'])
    tilt_deg = float(cfg.get('tilt_about_handle_deg', 0.))
    if not np.isfinite(tilt_deg) or abs(tilt_deg) > cfg.get('max_tilt_deg', 25.):
        raise Reject('tilt exceeds configured range')
    tilt_axis = unit(fit['axis']-vertical*np.dot(fit['axis'], vertical))
    angle = np.radians(tilt_deg)
    z = unit(vertical*np.cos(angle)+np.cross(tilt_axis, vertical)*np.sin(angle))
    x = fit['axis']
    if abs(np.dot(x, z)) > cfg['max_axis_approach_dot']:
        raise Reject('rod axis incompatible with top approach')
    if np.dot(fit['normal'], -vertical) < cfg.get('min_face_vertical_dot', 0.5):
        raise Reject('visible face is not the expected top face')
    if np.dot(fit['normal'], -z) < np.cos(np.radians(cfg.get('max_face_approach_angle_deg', 45.))):
        raise Reject('tilted approach too oblique to upper face')
    x = unit(x-z*np.dot(x, z))
    y = unit(np.cross(z, x))
    r = np.column_stack((x, y, z))
    center = fit['surface_center'] - fit['normal']*cfg['thickness_m']/2
    return dict(center_m=center.tolist(), grasp_point_m=center.tolist(),
                nominal_vertical_approach_camera=vertical.tolist(), approach_camera=z.tolist(),
                tilt_about_handle_deg=tilt_deg,
                rotation_matrix=r.tolist(), quaternion_xyzw=Rotation.from_matrix(r).as_quat().tolist(),
                convention='R_camera_tool columns: X along rod, Y finger closing, Z approach; TCP offset not applied',
                center_model='visible upper face minus local outward normal * physical thickness / 2')


def make_pose(fit, cfg):
    """Prefer vertical; use the smallest geometric tilt only if required.

    This policy checks the measured face and axis, not unknown gripper geometry.
    A collision-free robot approach cannot be certified by this function.
    """
    if cfg.get('approach_policy', 'fixed_test') != 'vertical_first':
        result = _make_pose_at_tilt(fit, cfg)
        result.update(selection_policy='explicit_angle_geometry_only', clearance_checked=False)
        return result
    angles = [0.]
    candidates = cfg.get('tilt_candidates_deg', [5, -5, 10, -10, 15, -15, 20, -20, 25, -25])
    if any(not np.isfinite(v) for v in candidates):
        raise Reject('invalid tilt candidates')
    angles.extend(sorted(set(float(v) for v in candidates if v != 0), key=abs))
    failures = []
    for angle in angles:
        attempt = dict(cfg, tilt_about_handle_deg=angle)
        try:
            result = _make_pose_at_tilt(fit, attempt)
            result.update(selection_policy='vertical_first_geometry_only', clearance_checked=False,
                          selection_reason='vertical satisfies face/axis geometry' if angle == 0 else
                          'smallest configured tilt satisfying face/axis geometry')
            return result
        except Reject as exc:
            if str(exc) not in ('tilted approach too oblique to upper face',
                                 'rod axis incompatible with top approach',
                                 'tilt exceeds configured range'):
                raise
            failures.append(str(exc))
    raise Reject('no vertical or small-tilt geometric approach: '+failures[0])


def fit_curved_handle(points, cfg, measurement_points=None):
    """Slightly bowed rectangular strip: z = a + b*u + c*v + d*u².

    Uses native raw measurements, RANSAC and robust refinement. This is an
    upper-face model, not a cylinder and not an arbitrary free-form surface.
    """
    from scipy.optimize import least_squares
    if len(points) < cfg['min_points']:
        raise Reject('insufficient curved-face points')
    anchor = np.median(points, axis=0)
    xy = points[:, :2] - anchor[:2]
    _, _, vt = np.linalg.svd(xy, full_matrices=False)
    e = vt[0]
    if e[np.argmax(np.abs(e))] < 0:
        e = -e
    u_axis = np.array([e[0], e[1], 0.])
    v_axis = np.array([-e[1], e[0], 0.])
    u, v = (points-anchor) @ u_axis, (points-anchor) @ v_axis
    depth = points[:, 2]-anchor[2]
    su, sv = max(np.std(u), .001), max(np.std(v), .001)
    design = np.column_stack((np.ones(len(points)), u/su, v/sv, (u/su)**2))
    rng = np.random.default_rng(31)
    sample = np.arange(len(points))
    if len(sample) > 1800:
        sample = rng.choice(sample, 1800, replace=False)
    a, b = design[sample], depth[sample]
    best = np.zeros(len(sample), bool)
    for _ in range(100):
        ids = rng.choice(len(sample), 4, replace=False)
        if np.linalg.cond(a[ids]) > 1e4:
            continue
        coeff = np.linalg.solve(a[ids], b[ids])
        inside = abs(a @ coeff-b) < cfg['plane_inlier_m']
        if inside.sum() > best.sum():
            best = inside
    if best.sum() < cfg['min_points']:
        raise Reject('curved-face RANSAC insufficient support')
    coeff = np.linalg.lstsq(a[best], b[best], rcond=None)[0]
    coeff = least_squares(lambda c: a @ c-b, coeff, loss='soft_l1', f_scale=.001).x
    dzdu = coeff[1]/su + 2*coeff[3]*u/su**2
    dzdv = coeff[2]/sv
    dist = (design @ coeff-depth)/np.sqrt(1+dzdu**2+dzdv**2)
    good = abs(dist) < cfg['plane_inlier_m']
    if good.mean() < cfg['min_plane_inlier_ratio']:
        raise Reject('curved-face inlier ratio %.3f below %.3f' % (good.mean(), cfg['min_plane_inlier_ratio']))
    coeff = least_squares(lambda c: design[good] @ c-depth[good], coeff,
                          loss='soft_l1', f_scale=.001).x
    dzdu = coeff[1]/su + 2*coeff[3]*u/su**2
    dist = (design @ coeff-depth)/np.sqrt(1+dzdu**2+(coeff[2]/sv)**2)
    good = abs(dist) < cfg['plane_inlier_m']
    if good.mean() < cfg['min_plane_inlier_ratio']:
        raise Reject('refined curved-face inlier ratio too low')
    core_inlier_ratio = float(good.mean())
    if measurement_points is not None:
        # Fit the interior to avoid L515 flying pixels at foreground boundaries,
        # then recover dimensions from ALL measured surface inliers. Erosion
        # must not silently shrink the reported width/length.
        points = measurement_points
        u, v = (points-anchor) @ u_axis, (points-anchor) @ v_axis
        depth = points[:, 2]-anchor[2]
        design = np.column_stack((np.ones(len(points)), u/su, v/sv, (u/su)**2))
        dzdu = coeff[1]/su + 2*coeff[3]*u/su**2
        dist = (design @ coeff-depth)/np.sqrt(1+dzdu**2+(coeff[2]/sv)**2)
        good = abs(dist) < cfg['plane_inlier_m']
        if good.mean() < cfg.get('min_raw_surface_support', .7):
            raise Reject('curved-face raw support too low')
    rmse = float(np.sqrt(np.mean(dist[good]**2)))
    if rmse > cfg['max_plane_rmse_m']:
        raise Reject('curved-face residual too large: %.4f m' % rmse)
    ug, vg = u[good], v[good]
    lo, hi = np.percentile(ug, [.5, 99.5])
    edges = np.linspace(lo, hi, 13)
    mids, lateral, widths = [], [], []
    for left, right in zip(edges[:-1], edges[1:]):
        chosen = (ug >= left) & (ug <= right)
        if chosen.sum() < max(3, good.sum()*.012):
            raise Reject('curved-face incomplete axial coverage')
        vl, vh = np.percentile(vg[chosen], [2, 98])
        mids.append((left+right)/2)
        lateral.append((vl+vh)/2)
        widths.append(vh-vl)
    # Centre of the visible ribbon across its width; allow mild image-plane bow.
    lateral_coeff = np.polyfit(np.asarray(mids)/su, lateral, 2)
    samples = np.linspace(lo, hi, 100)
    vv = np.polyval(lateral_coeff, samples/su)
    zz = coeff[0]+coeff[1]*samples/su+coeff[2]*vv/sv+coeff[3]*(samples/su)**2
    centerline = anchor + samples[:, None]*u_axis + vv[:, None]*v_axis
    centerline[:, 2] += zz
    segment_lengths = np.linalg.norm(np.diff(centerline, axis=0), axis=1)
    distance = np.r_[0., np.cumsum(segment_lengths)]
    length = float(distance[-1])
    mid_u = float(np.interp(length/2, distance, samples))
    mid_v = float(np.polyval(lateral_coeff, mid_u/su))
    surface = anchor + mid_u*u_axis + mid_v*v_axis
    surface[2] += coeff[0]+coeff[1]*mid_u/su+coeff[2]*mid_v/sv+coeff[3]*(mid_u/su)**2
    du = coeff[1]/su+2*coeff[3]*mid_u/su**2
    dv = coeff[2]/sv
    lateral_slope = (2*lateral_coeff[0]*mid_u/su+lateral_coeff[1])/su
    axis = unit(u_axis+lateral_slope*v_axis+np.array([0., 0., du+dv*lateral_slope]))
    normal = unit(du*u_axis+dv*v_axis-np.array([0., 0., 1.]))
    across = v_axis+np.array([0., 0., dv])
    across -= axis*np.dot(across, axis)
    width = float(np.median(widths)*np.linalg.norm(across))
    chord = unit(centerline[-1]-centerline[0])
    delta = centerline-centerline[0]
    sag = float(np.max(np.linalg.norm(delta-np.outer(delta @ chord, chord), axis=1)))
    if sag > cfg.get('max_bow_sag_m', .015):
        raise Reject('bow exceeds allowed near-straight model: %.4f m' % sag)
    if length < .001 or width < .001 or length/width < 3:
        raise Reject('invalid curved ribbon aspect ratio')
    if abs(length-cfg['length_m']) > cfg['length_tolerance_m']:
        raise Reject('curved length mismatch: %.4f m' % length)
    if abs(width-cfg['width_m']) > cfg['width_tolerance_m']:
        raise Reject('curved width mismatch: %.4f m' % width)
    return dict(surface_center=surface, axis=axis, normal=normal, length=length,
                width=width, rmse=rmse, inlier_ratio=float(good.mean()), points=points[good],
                bow_sag_m=sag, fit_model='quadratic_rectangular_upper_face',
                centerline=centerline,
                centerline_normals=np.array([unit(
                    (coeff[1]/su+2*coeff[3]*sample/su**2)*u_axis
                    + (coeff[2]/sv)*v_axis-np.array([0., 0., 1.])) for sample in samples]),
                core_inlier_ratio=core_inlier_ratio)
