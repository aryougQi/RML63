"""Small, standalone librealsense 2.50 C-ABI reader. No ROS or robot imports.

ABI definitions follow Intel's Apache-2.0 SDK headers at tag v2.50.0.
Frames are copied before release. Alignment uses SDK align, with no filling/filtering.
"""
import ctypes as C
from pathlib import Path
import time

import numpy as np

P, I, F, E = C.c_void_p, C.c_int, C.c_float, C.POINTER(C.c_void_p)


class Intrinsics(C.Structure):
    _fields_ = [('width', I), ('height', I), ('ppx', F), ('ppy', F),
                ('fx', F), ('fy', F), ('model', I), ('coeffs', F * 5)]


class Extrinsics(C.Structure):
    _fields_ = [('rotation', F * 9), ('translation', F * 3)]


class RealSense:
    def __init__(self, library):
        self.library = str(Path(library).expanduser().resolve())
        self.lib = C.CDLL(self.library)
        self.functions = {}
        signatures = {
            'get_api_version': (I, []), 'create_context': (P, [I]),
            'query_devices': (P, [P]), 'get_device_count': (I, [P]),
            'create_device': (P, [P, I]), 'get_device_info': (C.c_char_p, [P, I]),
            'supports_device_info': (I, [P, I]),
            'query_sensors': (P, [P]), 'get_sensors_count': (I, [P]),
            'create_sensor': (P, [P, I]), 'get_stream_profiles': (P, [P]),
            'get_stream_profiles_count': (I, [P]), 'get_stream_profile': (P, [P, I]),
            'get_stream_profile_data': (None, [P] + [C.POINTER(I)] * 5),
            'get_video_stream_intrinsics': (None, [P, C.POINTER(Intrinsics)]),
            'get_extrinsics': (None, [P, P, C.POINTER(Extrinsics)]),
            'create_pipeline': (P, [P]), 'create_config': (P, []),
            'config_enable_device': (None, [P, C.c_char_p]),
            'config_enable_stream': (None, [P, I, I, I, I, I, I]),
            'pipeline_start_with_config': (P, [P, P]),
            'pipeline_profile_get_device': (P, [P]), 'pipeline_stop': (None, [P]),
            'pipeline_wait_for_frames': (P, [P, C.c_uint]),
            'embedded_frames_count': (I, [P]), 'extract_frame': (P, [P, I]),
            'get_frame_stream_profile': (P, [P]), 'get_frame_data': (P, [P]),
            'get_frame_stride_in_bytes': (I, [P]),
            'get_frame_number': (C.c_ulonglong, [P]),
            'get_frame_timestamp': (C.c_double, [P]),
            'get_frame_timestamp_domain': (I, [P]),
            'depth_frame_get_units': (F, [P]), 'frame_add_ref': (None, [P]),
            'create_align': (P, [I]), 'create_frame_queue': (P, [I]),
            'start_processing_queue': (None, [P, P]), 'process_frame': (None, [P, P]),
            'wait_for_frame': (P, [P, C.c_uint]),
            'supports_frame_metadata': (I, [P, I]),
            'get_frame_metadata': (C.c_longlong, [P, I]),
        }
        for name, (result, args) in signatures.items():
            fn = getattr(self.lib, 'rs2_' + name)
            fn.restype, fn.argtypes = result, args + [E]
            self.functions[name] = fn
        for name in ['get_error_message', 'get_failed_function']:
            fn = getattr(self.lib, 'rs2_' + name)
            fn.restype, fn.argtypes = C.c_char_p, [P]
        self.lib.rs2_free_error.argtypes = [P]
        self.lib.rs2_free_error.restype = None
        for name in ['delete_context', 'delete_device_list', 'delete_device', 'delete_sensor_list',
                     'delete_sensor', 'delete_stream_profiles_list', 'delete_pipeline', 'delete_config',
                     'delete_pipeline_profile', 'release_frame', 'delete_processing_block', 'delete_frame_queue']:
            fn = getattr(self.lib, 'rs2_' + name)
            fn.argtypes, fn.restype = [P], None
        for name in ['stream', 'format', 'distortion', 'timestamp_domain', 'frame_metadata']:
            fn = getattr(self.lib, 'rs2_' + name + '_to_string')
            fn.argtypes, fn.restype = [I], C.c_char_p
        self.api_version = self.call('get_api_version')
        if self.api_version != 25000:
            raise ValueError('This audited L515 reader requires SDK 2.50.0; found %s. '
                             'Pass --sdk-library pointing to a compatible installation.' % self.api_version)
        self.context = self.call('create_context', self.api_version)
        self.pipeline = self.config = self.profile = self.align = self.queue = None

    def call(self, name, *args):
        error = P()
        value = self.functions[name](*args, C.byref(error))
        if error.value:
            try:
                message = self.lib.rs2_get_error_message(error).decode('utf-8', 'replace')
                function = self.lib.rs2_get_failed_function(error).decode('utf-8', 'replace')
            finally:
                self.lib.rs2_free_error(error)
            raise RuntimeError('%s: %s' % (function, message))
        return value

    def delete(self, name, handle):
        if handle:
            getattr(self.lib, 'rs2_' + name)(handle)

    def label(self, name, value):
        result = getattr(self.lib, 'rs2_' + name + '_to_string')(value)
        return result.decode('utf-8', 'replace') if result else str(value)

    def device_info(self, device):
        result = {}
        for key, index in [('name', 0), ('serial', 1), ('firmware', 2), ('physical_port', 4),
                           ('product_id', 7), ('usb_type', 9), ('product_line', 10)]:
            if self.call('supports_device_info', device, index):
                result[key] = self.call('get_device_info', device, index).decode('utf-8', 'replace')
        return result

    def stream_data(self, profile, intrinsics=True):
        fields = [I() for _ in range(5)]
        self.call('get_stream_profile_data', profile, *[C.byref(v) for v in fields])
        stream, fmt, index, uid, fps = [v.value for v in fields]
        result = dict(stream=self.label('stream', stream), format=self.label('format', fmt),
                      stream_index=index, unique_id=uid, fps=fps)
        if intrinsics:
            native = Intrinsics()
            self.call('get_video_stream_intrinsics', profile, C.byref(native))
            result.update(resolution_wh=[native.width, native.height], intrinsics=dict(
                width=native.width, height=native.height, fx=native.fx, fy=native.fy,
                ppx=native.ppx, ppy=native.ppy, distortion_model=self.label('distortion', native.model),
                distortion_model_id=native.model, coefficients=list(native.coeffs),
                coordinate_system='optical: x right, y down, z forward', source='active_sdk_stream_profile'))
        return result

    def probe(self):
        devices = self.call('query_devices', self.context)
        result = dict(sdk_library=self.library, sdk_api_version=self.api_version, devices=[])
        try:
            for i in range(self.call('get_device_count', devices)):
                device = self.call('create_device', devices, i)
                try:
                    item = self.device_info(device)
                    item['profiles'] = []
                    sensors = self.call('query_sensors', device)
                    try:
                        for j in range(self.call('get_sensors_count', sensors)):
                            sensor = self.call('create_sensor', sensors, j)
                            profiles = None
                            try:
                                profiles = self.call('get_stream_profiles', sensor)
                                for k in range(self.call('get_stream_profiles_count', profiles)):
                                    p = self.call('get_stream_profile', profiles, k)
                                    info = self.stream_data(p, intrinsics=False)
                                    if info['stream'] in ('Depth', 'Color'):
                                        info = self.stream_data(p)
                                        item['profiles'].append(info)
                            finally:
                                self.delete('delete_stream_profiles_list', profiles)
                                self.delete('delete_sensor', sensor)
                    finally:
                        self.delete('delete_sensor_list', sensors)
                    result['devices'].append(item)
                finally:
                    self.delete('delete_device', device)
        finally:
            self.delete('delete_device_list', devices)
        return result

    def start(self, config, serial=None):
        probe = self.probe()
        devices = [d for d in probe['devices'] if 'L515' in d.get('name', '') and (not serial or d['serial'] == serial)]
        if len(devices) != 1:
            raise RuntimeError('Exactly one selected L515 required; found %d. Use --serial when needed.' % len(devices))
        self.pipeline = self.call('create_pipeline', self.context)
        self.config = self.call('create_config')
        self.call('config_enable_device', self.config, devices[0]['serial'].encode())
        # rs2_stream: DEPTH=1, COLOR=2; rs2_format: Z16=1, RGB8=5 (SDK 2.50 ABI).
        for name, stream, fmt in [('depth', 1, 1), ('color', 2, 5)]:
            width, height = config[name + '_resolution_wh']
            self.call('config_enable_stream', self.config, stream, -1, width, height, fmt, config['fps'])
        self.profile = self.call('pipeline_start_with_config', self.pipeline, self.config)
        self.align = self.call('create_align', 2)
        self.queue = self.call('create_frame_queue', 1)
        self.call('start_processing_queue', self.align, self.queue)
        return dict(sdk_library=self.library, sdk_api_version=self.api_version, device=devices[0])

    def frame_parts(self, composite):
        result = {}
        try:
            for i in range(self.call('embedded_frames_count', composite)):
                frame = self.call('extract_frame', composite, i)
                try:
                    profile = self.call('get_frame_stream_profile', frame)
                    stream = self.stream_data(profile, intrinsics=False)['stream']
                except Exception:
                    self.delete('release_frame', frame)
                    raise
                if stream in ('Depth', 'Color') and stream not in result:
                    result[stream] = frame
                else:
                    self.delete('release_frame', frame)
            return result
        except Exception:
            for frame in result.values():
                self.delete('release_frame', frame)
            raise

    def copy_frame(self, frame):
        profile = self.call('get_frame_stream_profile', frame)
        info = self.stream_data(profile)
        width, height = info['resolution_wh']
        fmt = info['format']
        if fmt not in ('Z16', 'RGB8'):
            raise ValueError('Unsupported actual frame format: ' + fmt)
        stride = self.call('get_frame_stride_in_bytes', frame)
        row_bytes = width * (2 if fmt == 'Z16' else 3)
        if stride < row_bytes:
            raise ValueError('Invalid SDK frame stride')
        raw = C.string_at(self.call('get_frame_data', frame), stride * height)
        data = np.frombuffer(raw, dtype=np.uint8).reshape(height, stride)[:, :row_bytes].copy()
        array = data.view('<u2').reshape(height, width) if fmt == 'Z16' else data.reshape(height, width, 3)
        domain = self.call('get_frame_timestamp_domain', frame)
        info.update(frame_number=self.call('get_frame_number', frame),
                    timestamp_ms=self.call('get_frame_timestamp', frame),
                    timestamp_domain=self.label('timestamp_domain', domain), timestamp_domain_id=domain,
                    sdk_stride_bytes=stride, saved_dtype=str(array.dtype))
        info['sensor_metadata'] = {}
        # Values are copied verbatim; units follow the SDK enum documentation.
        for index in [0, 1, 2, 3, 4, 5, 9, 10, 11]:
            if self.call('supports_frame_metadata', frame, index):
                info['sensor_metadata'][self.label('frame_metadata', index)] = self.call('get_frame_metadata', frame, index)
        if fmt == 'Z16':
            info['depth_scale_m_per_unit'] = self.call('depth_frame_get_units', frame)
        return array, info, profile

    def next_pair(self, timeout_ms=5000):
        composite = self.call('pipeline_wait_for_frames', self.pipeline, timeout_ms)
        parts, aligned_parts, aligned = {}, {}, None
        try:
            parts = self.frame_parts(composite)
            if set(parts) != {'Depth', 'Color'}:
                raise RuntimeError('SDK frameset lacks color or depth')
            depth, dm, dp = self.copy_frame(parts['Depth'])
            color, cm, cp = self.copy_frame(parts['Color'])
            extrinsic = Extrinsics()
            self.call('get_extrinsics', dp, cp, C.byref(extrinsic))
            rotation = np.array(list(extrinsic.rotation)).reshape(3, 3, order='F')
            self.call('frame_add_ref', composite)
            self.call('process_frame', self.align, composite)  # consumes the added reference
            aligned = self.call('wait_for_frame', self.queue, timeout_ms)
            aligned_parts = self.frame_parts(aligned)
            aligned_depth, am, _ = self.copy_frame(aligned_parts['Depth'])
            metadata = dict(color=cm, depth_raw=dm, depth_aligned_to_color=am,
                depth_to_color=dict(rotation_column_major=list(extrinsic.rotation),
                    rotation_matrix=rotation.tolist(), translation_m=list(extrinsic.translation),
                    convention='P_color = R_depth_to_color @ P_depth + t_depth_to_color'),
                host_receive_unix_ns=time.time_ns(), host_receive_monotonic_ns=time.monotonic_ns())
            return color, depth, aligned_depth, metadata
        finally:
            for frame in list(parts.values()) + list(aligned_parts.values()):
                self.delete('release_frame', frame)
            self.delete('release_frame', aligned)
            self.delete('release_frame', composite)

    def close(self):
        if self.profile:
            try:
                self.call('pipeline_stop', self.pipeline)
            except RuntimeError:
                pass
        for name, attr in [('delete_processing_block', 'align'), ('delete_frame_queue', 'queue'),
                           ('delete_pipeline_profile', 'profile'), ('delete_config', 'config'),
                           ('delete_pipeline', 'pipeline'), ('delete_context', 'context')]:
            self.delete(name, getattr(self, attr, None))
            setattr(self, attr, None)
