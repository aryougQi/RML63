"""Runtime environment geometry validation; no capture or camera pipeline."""
from copy import deepcopy
import time
import numpy as np
from grasp_geometry import validate


def environment_geometry(config, now=None, require_verified=False):
    """Environment evidence independent of visual support-plane detection.

    Observed-depth mode validates geometry without asserting unseen clearance.
    Legacy modes retain time-bounded operator evidence. RobotSession checks the
    nonempty observed scene and scene consistency separately before execution.
    """
    now=time.time() if now is None else now
    data=deepcopy(config.get('environment_geometry') or {})
    if not data:
        data=dict(frame_id=config['base_frame'],boxes=[],verified=False,source='partial_depth_only')
    if data.get('frame_id')!=config['base_frame']:
        raise ValueError('environment geometry frame mismatch')
    mode=data.get('mode','measured_boxes')
    if mode not in ('measured_boxes','observed_depth_operator_confirmed','observed_depth'):
        raise ValueError('unsupported environment verification mode')
    observed_confirmed=(mode=='observed_depth_operator_confirmed' and
                        data.get('operator_clearance_confirmed') is True)
    boxes=data.setdefault('boxes',[])
    if not isinstance(boxes,list):raise ValueError('environment boxes must be a list')
    for box in boxes:
        validate(np.asarray(box['pose_base'],float))
        size=np.asarray(box['size_m'],float)
        if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):
            raise ValueError('invalid environment box dimensions')
    if config.get('environment_verification_enabled',True) is False:
        # Preview keeps actual scene geometry but cannot inherit a stale or
        # previously saved assertion of execution clearance.
        data.update(verified=False,operator_clearance_confirmed=False)
        if require_verified:
            raise RuntimeError('environment geometry unverified: verification disabled by operator; only detection/planning preview allowed; real execution remains disabled')
        return data
    if mode=='observed_depth':
        # Collision geometry is still validated above and by RobotSession.
        # Do not claim unseen space is verified, or reuse old confirmation.
        data.update(verified=False,operator_clearance_confirmed=False)
        data.pop('verified_at_unix',None)
        data.pop('valid_until_unix',None)
        return data
    verified=data.get('verified') is True
    if verified:
        max_age=float(config.get('environment_verification_max_age_sec',300.))
        if not np.isfinite(max_age) or max_age<=0:raise ValueError('invalid environment verification age')
        times=np.asarray([data.get('verified_at_unix',np.nan),data.get('valid_until_unix',np.nan)],float)
        if (not np.isfinite(times).all() or not times[0]<=now<times[1] or
            times[1]-times[0]>max_age or
            (not boxes and not observed_confirmed) or not data.get('source') or
            (mode=='observed_depth_operator_confirmed' and not observed_confirmed)):
            raise ValueError('environment verification missing, expired or invalid')
    if require_verified and not verified:
        raise RuntimeError('environment geometry unverified; only detection/planning preview allowed; configuration rejected before capture/planning')
    return data
