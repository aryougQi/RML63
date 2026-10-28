"""Validated planar PnP in OpenCV optical coordinates, no robot side effects.

T_camera_object maps object metres into camera metres: Xc = R @ Xo + t.
Camera axes: x right, y down, z forward. Image ordering is never a physical mapping.
"""
import cv2
import numpy as np


def _array(value, shape, name):
    try:a=np.asarray(value,dtype=np.float64)
    except (TypeError,ValueError) as exc:raise ValueError(f'{name}: numeric data required') from exc
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError(f'{name}: expected finite {shape}')
    return a


def _size(value,name):
    a=_array(value,(2,),name)
    if (a<1).any() or not np.equal(a,np.floor(a)).all():raise ValueError(f'{name}: positive integer [width,height] required')
    return a


def rectangle_model(length_m,width_m):
    try:d=np.array([length_m,width_m],dtype=float)
    except (TypeError,ValueError) as exc:raise ValueError('Measured length_m and width_m required') from exc
    if d.shape!=(2,) or not np.isfinite(d).all() or (d<=0).any():raise ValueError('Measured positive length_m and width_m required')
    x,y=d/2
    return np.array([[-x,-y,0],[x,-y,0],[x,y,0],[-x,y,0]],dtype=np.float64)


def _transform_points(points,h):
    homogeneous=np.column_stack((points,np.ones(len(points))))@h.T
    if (np.abs(homogeneous[:,2])<1e-10).any():raise ValueError('Image transform maps a corner to infinity')
    return homogeneous[:,:2]/homogeneous[:,2,None]


def _rotation_distance(a,b):
    ra=cv2.Rodrigues(a)[0];rb=cv2.Rodrigues(b)[0]
    return float(np.degrees(np.arccos(np.clip((np.trace(ra@rb.T)-1)/2,-1,1))))


def estimate_pose(object_points,image_points,camera_matrix,dist_coeffs,*,
                  physical_corner_mapping=None,input_geometry=None,
                  calibration_resolution_wh=None,length_unit='m',
                  max_reprojection_rmse_px=2.,max_point_error_px=4.,
                  ambiguity_margin_px=.5):
    """Return success/pose or rejected alternatives; invalid prerequisites raise ValueError.

    physical_corner_mapping = {confirmed: true, model_to_image_indices: [..], basis: ..}
    input_geometry = {confirmed: true, status: native|transformed,
      source_resolution_wh: [w,h], input_resolution_wh: [w,h], source_to_input_3x3: H}
    K/distortion always describe SOURCE pixels. Input corners are mapped back before PnP.
    This solves one explicitly supplied correspondence; it cannot infer symmetric orientation.
    """
    cv2.setNumThreads(1);cv2.setRNGSeed(0)
    if length_unit!='m':raise ValueError('Object coordinates must be metres (length_unit=m)')
    obj=_array(object_points,(4,3),'object_points')
    image=_array(image_points,(4,2),'image_points')
    if len(np.unique(obj,axis=0))!=4 or len(np.unique(image,axis=0))!=4:raise ValueError('Duplicate object/image corners')
    _,s,_=np.linalg.svd(obj-obj.mean(axis=0))
    if s[1]<1e-8 or s[2]>max(1e-8,s[0]*1e-6):raise ValueError('Four noncollinear coplanar object points required')
    if not cv2.isContourConvex(image.astype(np.float32)):raise ValueError('Image points must be a cyclic convex quadrilateral, not self intersecting')
    if abs(cv2.contourArea(image.astype(np.float32)))<16:raise ValueError('Projected quadrilateral too small for PnP')
    mapping=physical_corner_mapping
    if not isinstance(mapping,dict) or mapping.get('confirmed') is not True or not str(mapping.get('basis') or '').strip():
        raise ValueError('Confirmed physical corner mapping and its basis are required; image sorting is insufficient')
    indices=mapping.get('model_to_image_indices')
    if not isinstance(indices,list) or len(indices)!=4 or any(type(i) is not int for i in indices) or sorted(indices)!=list(range(4)):
        raise ValueError('model_to_image_indices must be a permutation of [0,1,2,3]')
    k=_array(camera_matrix,(3,3),'camera_matrix')
    resolution=_size(calibration_resolution_wh,'calibration_resolution_wh')
    if k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]) or abs(k[0,1])+abs(k[1,0])>1e-10:
        raise ValueError('Invalid OpenCV pinhole camera matrix (positive focal lengths, zero skew)')
    if dist_coeffs is None:raise ValueError('Explicit distortion coefficients required; use [] only for a verified rectified/pinhole image')
    dist=np.asarray(dist_coeffs,dtype=float).reshape(-1)
    if len(dist) not in (0,4,5,8,12,14) or not np.isfinite(dist).all():raise ValueError('Unsupported OpenCV distortion coefficients')
    geometry=input_geometry
    if not isinstance(geometry,dict) or geometry.get('confirmed') is not True or geometry.get('status') not in ('native','transformed'):
        raise ValueError('Unknown/unconfirmed crop or scale: cannot compute a calibrated pose')
    source_size=_size(geometry.get('source_resolution_wh'),'source_resolution_wh')
    input_size=_size(geometry.get('input_resolution_wh'),'input_resolution_wh')
    if not np.array_equal(source_size,resolution):raise ValueError('Calibration resolution must match the source camera image')
    h=_array(geometry.get('source_to_input_3x3'),(3,3),'source_to_input_3x3')
    if abs(np.linalg.det(h))<1e-10 or np.linalg.cond(h)>1e10:raise ValueError('Singular/unstable image transform')
    if geometry['status']=='native' and (not np.array_equal(input_size,source_size) or not np.allclose(h,np.eye(3))):
        raise ValueError('Native pixels require equal source/input resolution and identity transform')
    if ((image<0)|(image>=input_size)).any():raise ValueError('Corners outside input image')
    source=_transform_points(image,np.linalg.inv(h))
    if ((source<0)|(source>=source_size)).any():raise ValueError('Restored corners outside calibrated source image')
    observed=np.ascontiguousarray(image[indices]);source=np.ascontiguousarray(source[indices])
    options=np.asarray([max_reprojection_rmse_px,max_point_error_px,ambiguity_margin_px],dtype=float)
    if not np.isfinite(options).all() or (options[:2]<=0).any() or options[2]<0:raise ValueError('Invalid reprojection/ambiguity thresholds')

    def candidate(rv,tv):
        rv=np.asarray(rv,dtype=float).reshape(3,1);tv=np.asarray(tv,dtype=float).reshape(3,1)
        if not np.isfinite(rv).all() or not np.isfinite(tv).all():return None
        rotation=cv2.Rodrigues(rv)[0];camera=obj@rotation.T+tv.ravel()
        if (camera[:,2]<=1e-6).any():return None
        projected_source=cv2.projectPoints(obj,rv,tv,k,dist)[0].reshape(-1,2)
        projected=_transform_points(projected_source,h)
        error=np.linalg.norm(projected-observed,axis=1)
        if not np.isfinite(error).all():return None
        transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=tv.ravel()
        return {'rvec':rv.ravel().tolist(),'tvec_m':tv.ravel().tolist(),
                'rotation_matrix':rotation.tolist(),'T_camera_object':transform.tolist(),
                'reprojection_rmse_px':float(np.sqrt(np.mean(error**2))),
                'per_point_error_px':error.tolist(),'projected_input_points_model_order':projected.tolist(),
                'min_corner_depth_m':float(camera[:,2].min())}

    try:
        solved,rvecs,tvecs,_=cv2.solvePnPGeneric(np.ascontiguousarray(obj),source,k,dist,flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as exc:raise ValueError(f'OpenCV planar PnP failed: {exc}') from exc
    candidates=[]
    if solved:
        for rv,tv in zip(rvecs,tvecs):
            c=candidate(rv,tv)
            if c is None:continue
            if any(_rotation_distance(np.array(c['rvec']),np.array(other['rvec']))<.01 and np.linalg.norm(np.array(c['tvec_m'])-other['tvec_m'])<1e-7 for other in candidates):continue
            candidates.append(c)
    candidates.sort(key=lambda c:c['reprojection_rmse_px'])
    result={'success':False,'pose':None,'failure_reason':None,'method':'SOLVEPNP_IPPE + LM refinement',
            'alternatives':candidates,'length_unit':'m',
            'transform_convention':'X_camera = R_camera_object @ X_object + t_camera_object',
            'camera_axes':'x right, y down, z forward',
            'physical_corner_mapping':mapping,'input_geometry':geometry,
            'calibration_resolution_wh':resolution.astype(int).tolist(),
            'robot_ready':False,'depth_validated':False}
    if not candidates:result['failure_reason']='no_finite_positive_depth_solution';return result
    best=candidates[0]
    if len(candidates)>1 and candidates[1]['reprojection_rmse_px']-best['reprojection_rmse_px']<=ambiguity_margin_px:
        result['failure_reason']='ambiguous_planar_pose: IPPE alternatives have similar reprojection error';return result
    try:
        rv,tv=cv2.solvePnPRefineLM(obj,source,k,dist,np.array(best['rvec']).reshape(3,1),np.array(best['tvec_m']).reshape(3,1))
        refined=candidate(rv,tv)
        if refined and refined['reprojection_rmse_px']<=best['reprojection_rmse_px']:best=refined
    except cv2.error:
        pass  # Raw finite IPPE solution remains subject to the same reprojection gates.
    if best['reprojection_rmse_px']>max_reprojection_rmse_px or max(best['per_point_error_px'])>max_point_error_px:
        result['failure_reason']='reprojection_error_exceeds_threshold';return result
    result.update(success=True,pose=best)
    return result
