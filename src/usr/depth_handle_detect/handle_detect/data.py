import json
from pathlib import Path
import numpy as np


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def save_frame(path, depth, metadata):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    np.save(str(path / 'depth_raw.npy'), depth)
    write_json(path / 'metadata.json', metadata)


def load_frame(path):
    path = Path(path)
    m = json.loads((path / 'metadata.json').read_text())
    d = np.load(str(path / 'depth_raw.npy'), allow_pickle=False)
    k = m['intrinsics']
    if d.dtype != np.uint16 or d.ndim != 2:
        raise ValueError('raw depth must be a 2D uint16 array, never an 8-bit screenshot')
    if d.shape != (k['height'], k['width']):
        raise ValueError('depth grid and intrinsics mismatch')
    if not np.isfinite(m['depth_scale_m_per_unit']) or m['depth_scale_m_per_unit'] <= 0:
        raise ValueError('invalid depth scale')
    if not all(np.isfinite(k[x]) for x in ('fx', 'fy', 'ppx', 'ppy')) or min(k['fx'], k['fy']) <= 0:
        raise ValueError('invalid intrinsics')
    return d, m


def points_from_pixels(depth, metadata, mask):
    k = metadata['intrinsics']
    v, u = np.nonzero(mask & (depth > 0))
    z = depth[v, u].astype(np.float64) * metadata['depth_scale_m_per_unit']
    coeff = k.get('coefficients', [0.] * 5)
    model = k.get('distortion_model_id', 0)
    if model != 0 and np.any(coeff):
        import pyrealsense2 as rs
        intr = rs.intrinsics()
        for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
            setattr(intr, key, k[key])
        intr.model = rs.distortion(model)
        intr.coeffs = coeff
        return np.array([rs.rs2_deproject_pixel_to_point(intr, [int(x), int(y)], float(t))
                         for x, y, t in zip(u, v, z)])
    return np.column_stack(((u-k['ppx'])*z/k['fx'], (v-k['ppy'])*z/k['fy'], z))


def capture(output, count=30, width=640, height=480, fps=30, serial=None, on_frame=None,
            color_preview=False, min_distance_mm=None, warmup_processor=None):
    import pyrealsense2 as rs
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError('capture directory must be empty; preserve previous raw data')
    context = rs.context()
    devices = [d for d in context.query_devices()
               if 'L515' in d.get_info(rs.camera_info.name)
               and (serial is None or d.get_info(rs.camera_info.serial_number) == serial)]
    if len(devices) != 1:
        raise RuntimeError('expected one available L515; found %d (check USB ownership or specify serial)' % len(devices))
    device = devices[0]
    profiles = [p.as_video_stream_profile() for s in device.query_sensors()
                for p in s.get_stream_profiles() if p.stream_type() == rs.stream.depth
                and p.format() == rs.format.z16 and p.is_video_stream_profile()]
    supported = sorted(set((p.width(), p.height(), p.fps()) for p in profiles))
    if (width, height, fps) not in supported:
        raise ValueError('unsupported depth profile %dx%d@%d; available %s; USB %s' %
                         (width, height, fps, supported, device.get_info(rs.camera_info.usb_type_descriptor)))
    pipeline, cfg = rs.pipeline(context), rs.config()
    cfg.enable_device(device.get_info(rs.camera_info.serial_number))
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    if color_preview:
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)
    started = False
    sensor, previous_min, previous_preset = None, None, None
    try:
        profile = pipeline.start(cfg)
        started = True
        dev = profile.get_device()
        name = dev.get_info(rs.camera_info.name)
        if 'L515' not in name:
            raise ValueError('expected L515, got ' + name)
        sensor = dev.first_depth_sensor()
        if min_distance_mm is not None:
            limits = sensor.get_option_range(rs.option.min_distance)
            if not np.isfinite(min_distance_mm) or not max(250, limits.min) <= min_distance_mm <= limits.max:
                raise ValueError('min distance must be at least 250 mm and within SDK limits')
            previous_min = sensor.get_option(rs.option.min_distance)
            previous_preset = sensor.get_option(rs.option.visual_preset)
            sensor.set_option(rs.option.min_distance, min_distance_mm)
        # Hardware option queries can block long enough to skip frames on USB2.
        # These settings remain constant during this session: read them once.
        device_serial = dev.get_info(rs.camera_info.serial_number)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        sensor_options = {str(o): sensor.get_option(o) for o in
                          (rs.option.min_distance, rs.option.confidence_threshold, rs.option.visual_preset)}
        depth_profile = profile.get_stream(rs.stream.depth)
        color_profiles = [p for s in dev.query_sensors() for p in s.get_stream_profiles()
                          if p.stream_type() == rs.stream.color and p.is_video_stream_profile()]
        depth_to_color = None
        if color_profiles:
            color_profile = profile.get_stream(rs.stream.color) if color_preview else color_profiles[0]
            extrinsic = depth_profile.get_extrinsics_to(color_profile)
            depth_to_color = dict(rotation_matrix=np.array(extrinsic.rotation).reshape(3, 3, order='F').tolist(),
                                  translation_m=list(extrinsic.translation),
                                  convention='P_color_optical = R @ P_depth_optical + t',
                                  source='factory SDK extrinsics of the captured device')
        for warmup_index in range(30):
            warmup_frames = pipeline.wait_for_frames(5000)
            if warmup_index == 0 and warmup_processor is not None:
                warmup_depth = warmup_frames.get_depth_frame()
                k = warmup_depth.profile.as_video_stream_profile().intrinsics
                warmup_metadata = dict(intrinsics={key: getattr(k, key) for key in
                    ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')},
                    depth_scale_m_per_unit=float(warmup_depth.get_units()), origin='real_l515_warmup')
                warmup_metadata['intrinsics'].update(distortion_model_id=int(k.model), coefficients=list(k.coeffs))
                warmup_processor(np.asanyarray(warmup_depth.get_data()).copy(), warmup_metadata)
        last = None
        for i in range(count):
            frames = pipeline.wait_for_frames(5000)
            f = frames.get_depth_frame()
            if not f or f.get_frame_number() == last:
                raise RuntimeError('missing or duplicate depth frame')
            last = f.get_frame_number()
            k = f.profile.as_video_stream_profile().intrinsics
            m = dict(intrinsics={key: getattr(k, key) for key in
                     ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')},
                     depth_scale_m_per_unit=float(f.get_units()), frame_number=last,
                     timestamp_ms=f.get_timestamp(), timestamp_domain=str(f.get_frame_timestamp_domain()),
                     device=name, serial=device_serial,
                     origin='real_l515', coordinate_frame='depth_optical')
            m['intrinsics'].update(distortion_model_id=int(k.model), coefficients=list(k.coeffs))
            m['usb_type'] = usb_type
            m['sensor_options'] = sensor_options
            m['depth_to_color'] = depth_to_color
            raw = np.asanyarray(f.get_data()).copy()
            directory = output / ('frame_%06d' % i)
            if color_preview:
                color = frames.get_color_frame()
                if not color:
                    raise RuntimeError('color preview frame missing')
                m['color_preview'] = dict(file='rgb.png', aligned_to_depth=False,
                    frame_number=color.get_frame_number(), timestamp_ms=color.get_timestamp(),
                    note='Native color grid; use native depth grid for ROI and 3D')
            save_frame(directory, raw, m)
            if color_preview:
                import cv2
                if not cv2.imwrite(str(directory/'rgb.png'),
                        cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)):
                    raise OSError('could not save RGB preview')
            if on_frame is not None:
                on_frame(raw, m)
    finally:
        try:
            if previous_min is not None:
                try:
                    sensor.set_option(rs.option.min_distance, previous_min)
                finally:
                    # Setting a manual option switches the L515 to Custom. Restore
                    # the preset last so the caller's preset is also preserved.
                    sensor.set_option(rs.option.visual_preset, previous_preset)
        finally:
            if started:
                pipeline.stop()
