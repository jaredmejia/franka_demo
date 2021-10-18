import time
import numpy as np
import cv2
import pyrealsense2 as rs
from PIL import Image

from franka_demo.demo_interfaces import print_and_cr

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30

def add_camera_function(state):
    state.realsense = RealSense()
    state.handlers['d'] = debug_save_images

def debug_save_images(key_pressed, state):
    state.realsense.save_images_to_file('debug')

class RealSense:
    """ Wrapper that implements boilerplate code for RealSense cameras """

    def __init__(self):
        # TODO read from config
        self.device_ls = []
        for cam in rs.context().query_devices():
            self.device_ls.append(cam.get_info(rs.camera_info(1)))
        self.device_ls.sort()

        # Start streaming
        print_and_cr(f"Connecting to RealSense cameras ({len(self.device_ls)} found) ...")
        self.pipes = []
        self.profiles = []
        for i, device_id in enumerate(self.device_ls):
            pipe = rs.pipeline()
            config = rs.config()

            config.enable_device(device_id)
            config.enable_stream(rs.stream.depth, CAM_WIDTH, CAM_HEIGHT, rs.format.z16, CAM_FPS)
            config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)

            self.pipes.append(pipe)
            self.profiles.append(pipe.start(config))

            print_and_cr(f"Connected to camera {i+1} ({device_id}).")
            time.sleep(0.5)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        print_and_cr(f"[INFO] Warm start cameras (realsense auto-adjusts brightness during initial frames)")
        for _ in range(60): self._get_frames()
        print_and_cr(f"[INFO] Camera setup completed.")

    def get_num_cameras(self):
        return len(self.device_ls)

    def _get_frames(self):
        raw_frames = [pipe.wait_for_frames() for pipe in self.pipes]
        aligned_frames = [self.align.process(f) for f in raw_frames]
        return aligned_frames

    def get_data(self):
        framesets = self._get_frames()
        data = []
        for frameset in framesets:
            color_frame = frameset.get_color_frame()
            depth_frame = frameset.get_depth_frame()
            data.append((
                np.asanyarray(color_frame.get_data()),
                np.asanyarray(depth_frame.get_data())))
        return data

    def save_images_to_file(self, fn_prefix):
        data = self.get_data()
        for i, (color_image, depth_image) in enumerate(data):
            color_im = Image.fromarray(color_image[:,:,::-1])
            color_im.save(f"{fn_prefix}-cam{i}-color.jpeg")
            depth_im = Image.fromarray(depth_image.astype(np.uint8))
            depth_im.save(f"{fn_prefix}-cam{i}-depth.png")