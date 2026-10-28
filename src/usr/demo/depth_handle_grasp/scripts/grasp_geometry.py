"""Coordinate and EG2 linkage geometry; no ROS publishers or robot commands."""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from functools import lru_cache
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation


def rotation_matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, "as_matrix") else rotation.as_dcm()


def transform(xyz, rotation):
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation, dtype=float)
    t[:3, 3] = np.asarray(xyz, dtype=float)
    validate(t)
    return t


def validate(t):
    if np.asarray(t).shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('invalid homogeneous transform')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7) or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=2e-5) or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=2e-5):
        raise ValueError('invalid rigid rotation/transform')
    return t


def observed_obstacle_points(points, nearby, above, target_mask):
    """Return observed scene points excluding the object being grasped."""
    points = np.asarray(points)
    nearby = np.asarray(nearby, dtype=bool)
    above = np.asarray(above, dtype=bool)
    target_mask = np.asarray(target_mask, dtype=bool)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if any(mask.shape != (len(points),) for mask in (nearby, above, target_mask)):
        raise ValueError('point filters must have shape (N,)')
    # The open gripper is expected to approach the target closely. Only
    # non-target observed geometry is an environmental obstacle.
    return points[nearby & above & ~target_mask]


def collision_samples(points, target, voxel_m, detail_radius_m):
    """Preserve native obstacle locations near the grasp; voxelize farther away."""
    points=np.asarray(points,float);target=np.asarray(target,float)
    if points.ndim!=2 or points.shape[1]!=3 or target.shape!=(3,) or not np.isfinite(points).all() or not np.isfinite(target).all():
        raise ValueError('invalid observed collision samples')
    if not np.isfinite(voxel_m) or voxel_m<=0 or not np.isfinite(detail_radius_m) or detail_radius_m<0:
        raise ValueError('invalid collision sampling dimensions')
    detailed=np.linalg.norm(points-target,axis=1)<=detail_radius_m
    far=points[~detailed];near=points[detailed]
    centers=(np.unique(np.floor(far/voxel_m).astype(int),axis=0)+.5)*voxel_m if len(far) else np.empty((0,3))
    exact=np.unique(near,axis=0) if len(near) else np.empty((0,3))
    return centers,exact


def sample_bounds(groups):
    """Closed spheres contain every existing CAD surface sample in each group."""
    result=[]
    for points in groups:
        points=np.asarray(points,float)
        if not len(points):continue
        center=(points.min(0)+points.max(0))/2
        radius=float(np.linalg.norm(points-center,axis=1).max())+1e-12
        result.append((center,radius,points))
    return result


def bounded_sample_clearance(tree,groups,flange,margin):
    """Triangle inequality proves each pruned sample is >= margin from obstacles.

    No obstacle or CAD sample is removed. Ambiguous groups use the exact old
    nearest-neighbour check. Return a conservative lower bound for reporting.
    """
    if not groups:return float('inf'),0,0
    centers=np.asarray([g[0] for g in groups])@flange[:3,:3].T+flange[:3,3]
    distances=tree.query(centers,k=1)[0]
    minimum=float('inf');pruned=queried=0
    for distance,(_,radius,points) in zip(distances,groups):
        lower=float(distance)-radius
        if lower>=margin:
            pruned+=1;minimum=min(minimum,lower)
        else:
            transformed=points@flange[:3,:3].T+flange[:3,3]
            exact=tree.query(transformed,k=1)[0]
            minimum=min(minimum,float(np.min(exact)));queried+=len(points)
    return minimum,pruned,queried


def optical_to_base(corrected_base_flange, handeye_color, depth_to_color):
    # The hand-eye calibration is for COLOR optical, not the depth grid.
    native = transform(depth_to_color['translation_m'], depth_to_color['rotation_matrix'])
    return validate(corrected_base_flange) @ validate(handeye_color) @ native


def matrix_from_pose(pose):
    return transform(pose['grasp_point_m'], pose['rotation_matrix'])


def flange_target(base_tcp, flange_tcp):
    return validate(base_tcp) @ np.linalg.inv(validate(flange_tcp))


def offset_target(base_tcp, approach, distance):
    out = validate(base_tcp).copy()
    direction = np.asarray(approach, dtype=float)
    if direction.shape != (3,) or not np.isfinite(direction).all() or not np.isclose(np.linalg.norm(direction), 1., atol=1e-5):
        raise ValueError('invalid approach direction')
    out[:3, 3] -= direction*distance
    return out


def inward_grasp(base_tcp, surface_normal, rod_axis, angle_deg,
                 max_face_angle_deg=25., max_axis_approach_dot=.35, azimuth_deg=0.):
    """Pitch tool Z toward the arm base, retaining the measured grasp centre."""
    out = validate(base_tcp).copy()
    if not np.isfinite(angle_deg) or not 0 <= angle_deg <= 15:
        raise ValueError('inward grasp tilt must be within 0..15 degrees')
    if not np.isfinite(azimuth_deg) or abs(azimuth_deg)>90:
        raise ValueError('approach azimuth must be within -90..90 degrees of inward')
    if angle_deg == 0:
        return out
    normal, axis = np.asarray(surface_normal, float), np.asarray(rod_axis, float)
    for vector in (normal, axis):
        if vector.shape != (3,) or not np.isfinite(vector).all() or not np.isclose(np.linalg.norm(vector), 1., atol=1e-5):
            raise ValueError('invalid measured surface normal or rod axis')
    vertical = out[:3, 2]
    inward = -out[:3, 3]
    inward -= vertical*np.dot(inward, vertical)
    if np.linalg.norm(inward) < 1e-6:
        raise ValueError('inward direction undefined at the arm base')
    inward /= np.linalg.norm(inward)
    azimuth=np.radians(azimuth_deg)
    inward=inward*np.cos(azimuth)+np.cross(vertical,inward)*np.sin(azimuth)
    angle = np.radians(angle_deg)
    z = vertical*np.cos(angle)+inward*np.sin(angle)
    if abs(np.dot(axis, z)) > max_axis_approach_dot:
        raise ValueError('inward tilt incompatible with measured rod axis')
    if np.dot(normal, -z) < np.cos(np.radians(max_face_angle_deg)):
        raise ValueError('inward tilt too oblique to measured top face')
    x = axis-z*np.dot(axis, z)
    x /= np.linalg.norm(x)
    out[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    return validate(out)


def raised_pad_grasp(base_tcp, section_center, surface_normal, thickness_m,
                     pad_bounds_tcp, raise_m, edge_margin_m=.004):
    """Raise pad centres only while the wood section stays within both pads."""
    out=validate(base_tcp).copy()
    normal=np.asarray(surface_normal,float);center=np.asarray(section_center,float)
    if normal.shape!=(3,) or center.shape!=(3,) or not np.isfinite(normal).all() or not np.isfinite(center).all() or not np.isclose(np.linalg.norm(normal),1.,atol=1e-5):
        raise ValueError('invalid measured section geometry')
    if not np.isfinite(raise_m) or not 0<=raise_m<=.01 or thickness_m<=0 or not np.isfinite(thickness_m) or not np.isfinite(edge_margin_m) or edge_margin_m<0:
        raise ValueError('invalid pad contact height')
    out[:3,3]+=normal*raise_m
    n=out[:3,:3].T@normal;c=out[:3,:3].T@(center-out[:3,3])
    if abs(n[2])<.5:raise ValueError('wood surface too oblique for pad coverage')
    bounds=np.asarray(pad_bounds_tcp,float)
    if bounds.shape!=(2,2,3) or not np.isfinite(bounds).all() or np.any(bounds[:,1]<bounds[:,0]):
        raise ValueError('invalid CAD finger pad bounds')
    for low,high in bounds:
        mids=[c[2]-(n[0]*(x-c[0])+n[1]*(y-c[1]))/n[2]
              for x in [low[0],high[0]] for y in [low[1],high[1]]]
        half=thickness_m/(2*abs(n[2]))
        if min(mids)-half<low[2]+edge_margin_m or max(mids)+half>high[2]-edge_margin_m:
            raise ValueError('wood section would exceed effective finger pad contact area')
    return out


def stl_triangles(path):
    blob = Path(path).read_bytes()
    count = struct.unpack_from('<I', blob, 80)[0]
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    if len(blob) != 84+count*50:
        raise ValueError('expected binary STL')
    return np.frombuffer(blob, dtype=dtype, count=count, offset=84)


class GripperGeometry:
    def __init__(self, urdf, mesh_dir, mounting_xyz, mounting_rpy):
        root = ET.fromstring(Path(urdf).read_text())
        self.parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.mount = transform(mounting_xyz, rotation_matrix(Rotation.from_euler('xyz', mounting_rpy)))
        self.meshes = {}
        for link in root.findall('link'):
            name = link.get('name')
            self.meshes[name] = stl_triangles(Path(mesh_dir)/(name+'.STL'))
        self.pads = {};self.pad_faces={}
        for link, sign in [('4C2_Link5', 1), ('4C2_Link6', -1)]:
            triangle = self.meshes[link]
            xs = triangle['vertices'][:, :, 0]*sign
            face = ((triangle['normal'][:, 0]*sign > .99)
                    & (abs(xs-xs.max()).max(axis=1) < .0002))
            points = triangle['vertices'][face].reshape(-1, 3)
            if len(points) < 3:
                raise ValueError('cannot identify original finger contact pad')
            self.pads[link] = (points.min(0)+points.max(0))/2
            self.pad_faces[link]=points

    def link(self, name, angle):
        chain, current = [], name
        while current != '4C2_baselink':
            joint = self.parents[current]
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            t = transform(xyz, rotation_matrix(Rotation.from_euler('xyz', rpy)))
            if joint.get('type') != 'fixed':
                mimic = joint.find('mimic')
                multiplier = float(mimic.get('multiplier', '1')) if mimic is not None else 1.
                offset = float(mimic.get('offset', '0')) if mimic is not None else 0.
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                t[:3, :3] = t[:3, :3] @ rotation_matrix(Rotation.from_rotvec(axis*(angle*multiplier+offset)))
            chain.append(t); current = joint.find('parent').get('link')
        out = self.mount.copy()
        for t in reversed(chain):
            out = out @ t
        return out

    def pad_centers(self, angle):
        return np.array([(self.link(name, angle) @ np.r_[p, 1.])[:3] for name, p in self.pads.items()])

    def opening(self, angle):
        return float(np.linalg.norm(np.diff(self.pad_centers(angle), axis=0)[0]))

    def at_width(self,width):
        # Cache only immutable CAD kinematics, never IK or collision results.
        # Mount bytes participate in the key; callers receive independent arrays.
        result=self._at_width(float(width),self.mount.tobytes())
        return {key:value.copy() if isinstance(value,np.ndarray) else value for key,value in result.items()}

    @lru_cache(maxsize=256)
    def _at_width(self, width, mount_signature):
        if not np.isfinite(width) or not 0 <= width <= self.opening(.82):
            raise ValueError('opening outside CAD linkage range')
        angle = brentq(lambda a: self.opening(a)-width, 0, .82, xtol=1e-10) if width > 1e-6 else 0.
        pads = self.pad_centers(angle)
        # TCP X along the rod (gripper-base +Y), Y closing (base -X), Z approach.
        rotation = self.mount[:3, :3] @ rotation_matrix(Rotation.from_euler('z', np.pi/2))
        tcp = transform(pads.mean(0), rotation)
        bounds=[]
        for name,points in self.pad_faces.items():
            frame=self.link(name,angle)
            local=(points@frame[:3,:3].T+frame[:3,3]-tcp[:3,3])@tcp[:3,:3]
            bounds.append([local.min(0),local.max(0)])
        return dict(linkage_angle_rad=angle, opening_m=self.opening(angle), flange_tcp=tcp,
                    pad_centers_flange_m=pads,pad_bounds_tcp_m=np.asarray(bounds),
                    source='official original finger pad CAD + measured mounting transform')
