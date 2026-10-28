#!/usr/bin/env python3
"""Interactive native-depth ROI selector using Tk or a live ROS depth topic."""
import argparse
import base64
import json
from pathlib import Path
import tkinter as tk
import numpy as np
import cv2

HERE = Path(__file__).resolve().parents[1]


class RoiSelector:
    def __init__(self, image_path, config_path, write_config):
        self.image_path = Path(image_path)
        self.config_path = Path(config_path)
        self.write_config = write_config
        self.root = tk.Tk()
        self.root.title('Depth ROI selector')
        self.photo = tk.PhotoImage(file=str(self.image_path))
        self.image_width = self.photo.width()
        self.image_height = self.photo.height()
        self.canvas = tk.Canvas(self.root, width=self.image_width, height=self.image_height,
                                cursor='crosshair')
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw')
        self.status = tk.StringVar(value='Drag a box around the handle with 10-15 px margin')
        tk.Label(self.root, textvariable=self.status, anchor='w').pack(fill='x')
        buttons = tk.Frame(self.root)
        buttons.pack(fill='x')
        tk.Button(buttons, text='Save ROI', command=self.save).pack(side='left', padx=6, pady=6)
        tk.Button(buttons, text='Cancel', command=self.root.destroy).pack(side='right', padx=6, pady=6)
        self.start = None
        self.rectangle = None
        self.selection = None
        self.canvas.bind('<ButtonPress-1>', self.begin)
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<ButtonRelease-1>', self.end)

    def begin(self, event):
        self.start = (event.x, event.y)
        self.selection = None
        if self.rectangle is not None:
            self.canvas.delete(self.rectangle)
        self.rectangle = self.canvas.create_rectangle(event.x, event.y, event.x, event.y,
                                                      outline='red', width=2)

    def drag(self, event):
        if self.start is None:
            return
        x0, y0 = self.start
        self.canvas.coords(self.rectangle, x0, y0, event.x, event.y)

    def end(self, event):
        if self.start is None:
            return
        x0, y0 = self.start
        x1, y1 = event.x, event.y
        x, right = sorted((max(0, x0), min(self.image_width, x1)))
        y, bottom = sorted((max(0, y0), min(self.image_height, y1)))
        self.selection = [x, y, right - x, bottom - y]
        self.status.set('ROI: %s  click Save ROI to write the config' % self.selection)

    def save(self):
        if not self.selection or self.selection[2] < 3 or self.selection[3] < 3:
            self.status.set('ROI is too small; drag again')
            return
        print('roi_xywh =', self.selection)
        if self.write_config:
            config = json.loads(self.config_path.read_text())
            config['roi_xywh'] = self.selection
            self.config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n')
            print('Wrote', self.config_path)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def decode_depth_message(msg):
    if msg.encoding not in ('16UC1', 'mono16'):
        raise ValueError('unsupported depth encoding: %s' % msg.encoding)
    dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
    row_width = msg.step // 2
    data = np.frombuffer(msg.data, dtype=dtype)
    expected = msg.height * row_width
    if data.size < expected or row_width < msg.width:
        raise ValueError('invalid depth image stride or data length')
    return data[:expected].reshape(msg.height, row_width)[:, :msg.width].copy()


def depth_photo(depth):
    valid = depth > 0
    gray = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        values = depth[valid].astype(np.float32)
        lo, hi = np.percentile(values, [2, 98])
        gray[valid] = np.clip((values-lo)/max(hi-lo, 1.)*255., 0, 255).astype(np.uint8)
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    ok, encoded_png = cv2.imencode('.png', rgb)
    if not ok:
        raise RuntimeError('could not encode depth preview')
    encoded = base64.b64encode(encoded_png.tobytes()).decode('ascii')
    return tk.PhotoImage(data=encoded, format='PNG')


class DepthRoiSelector:
    def __init__(self, depth, config_path, write_config, support_plane=False):
        self.config_path = Path(config_path)
        self.write_config = write_config
        self.depth = depth
        self.support_plane = support_plane
        self.image_height, self.image_width = depth.shape
        self.root = tk.Tk()
        self.root.title('Current depth ROI selector')
        if support_plane:
            self.root.title('Select a confirmed horizontal reference surface')
        self.photo = depth_photo(depth)
        self.canvas = tk.Canvas(self.root, width=self.image_width, height=self.image_height,
                                cursor='crosshair')
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw')
        self.status = tk.StringVar(value='Current depth frame. Drag a box with 10-15 px margin.')
        if support_plane:
            self.status.set('Select bare HORIZONTAL floor/platform, excluding basket, cables and robot.')
        tk.Label(self.root, textvariable=self.status, anchor='w').pack(fill='x')
        buttons = tk.Frame(self.root)
        buttons.pack(fill='x')
        tk.Button(buttons, text='Save ROI', command=self.save).pack(side='left', padx=6, pady=6)
        tk.Button(buttons, text='Cancel', command=self.root.destroy).pack(side='right', padx=6, pady=6)
        self.start = None
        self.rectangle = None
        self.selection = None
        self.canvas.bind('<ButtonPress-1>', self.begin)
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<ButtonRelease-1>', self.end)

    def begin(self, event):
        self.start = (event.x, event.y)
        self.selection = None
        if self.rectangle is not None:
            self.canvas.delete(self.rectangle)
        self.rectangle = self.canvas.create_rectangle(event.x, event.y, event.x, event.y,
                                                      outline='red', width=2)

    def drag(self, event):
        if self.start is not None and self.rectangle is not None:
            self.canvas.coords(self.rectangle, self.start[0], self.start[1], event.x, event.y)

    def end(self, event):
        if self.start is None:
            return
        x0, y0 = self.start
        x1, y1 = event.x, event.y
        x, right = sorted((max(0, x0), min(self.image_width, x1)))
        y, bottom = sorted((max(0, y0), min(self.image_height, y1)))
        self.selection = [x, y, right-x, bottom-y]
        self.status.set('ROI: %s. Click Save ROI.' % self.selection)

    def save(self):
        if not self.selection or self.selection[2] < 3 or self.selection[3] < 3:
            self.status.set('ROI is too small; drag again')
            return
        print('roi_xywh =', self.selection)
        if self.write_config:
            config = json.loads(self.config_path.read_text())
            if self.support_plane:
                x, y, w, h = self.selection
                patch = self.depth[y:y+h, x:x+w]
                values = patch[patch > 0].astype(float) / 1000.
                reference = config['support_plane']
                if len(values) < reference.get('min_points', 300):
                    self.status.set('Not enough valid depth. Select a larger bare horizontal surface.')
                    return
                lo, hi = np.percentile(values, [1, 99])
                reference.update(roi_xywh=self.selection,
                                 depth_range_m=[max(.001, float(lo)-.02), float(hi)+.02],
                                 enabled=True, horizontal_confirmed=True)
            else:
                config['roi_xywh'] = self.selection
            self.config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n')
            print('Wrote', self.config_path)
        self.root.destroy()

    def run(self):
        self.root.mainloop()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path,
                        help='native-depth preview PNG, normally 320x240')
    parser.add_argument('--live', action='store_true',
                        help='capture one current ROS depth frame instead of a saved image')
    parser.add_argument('--topic', default='/camera/depth/image_rect_raw')
    parser.add_argument('--config', type=Path,
                        default=HERE/'configs/real_handle.json')
    parser.add_argument('--write-config', action='store_true',
                        help='write selected roi_xywh into --config')
    parser.add_argument('--support-plane', action='store_true',
                        help='select a confirmed horizontal reference ROI instead of the handle; requires --live')
    args = parser.parse_args()
    if args.support_plane and not args.live:
        parser.error('--support-plane requires --live')
    if args.live:
        import rospy
        from sensor_msgs.msg import Image as RosImage
        rospy.init_node('depth_roi_selector', anonymous=True)
        try:
            msg = rospy.wait_for_message(args.topic, RosImage, timeout=10.)
            depth = decode_depth_message(msg)
        except Exception as error:
            parser.error('cannot read current depth frame: %s' % error)
        DepthRoiSelector(depth, args.config, args.write_config, args.support_plane).run()
    else:
        if args.image is None:
            parser.error('--image is required unless --live is used')
        RoiSelector(args.image, args.config, args.write_config).run()


if __name__ == '__main__':
    main()
