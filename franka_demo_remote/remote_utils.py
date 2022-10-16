import time
import numpy as np
import cv2
from franka_demo_remote.utils import print_and_cr

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_KEYS = ["cam0c", "cam1c", "cam2c"]
FRAME_TYPE = np.uint8

def redis_send_frame(redis_store, cam_state):
    for key in CAM_KEYS:
        if key in cam_state:
            redis_store.set(key, cam_state[key][0].tobytes())

def redis_receive_frame(redis_store):
    cam_state = {}
    for key in CAM_KEYS:
        cam_state[key] = np.array(np.frombuffer(redis_store.get(key), dtype=FRAME_TYPE).reshape([CAM_HEIGHT, CAM_WIDTH, 3]))
    return cam_state

def render_cam_state(state):
    """ Update camera info"""
    print_and_cr("[INFO] Rendering in progress...")
    while not state.quit:
        cam_state = redis_receive_frame(state.redis_store)
        imgs_ti = []
        for i in range(len(CAM_KEYS)):
            color_image = cam_state[CAM_KEYS[i]]
            imgs_ti.append(color_image)
        stacked_imgs_ti = np.hstack(imgs_ti)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # img = cv2.resize(stacked_imgs_ti, (CAM_WIDTH * len(CAM_KEYS) * 2, CAM_HEIGHT * 2)) ## show twice as big 
        img = cv2.resize(stacked_imgs_ti, (CAM_WIDTH * len(CAM_KEYS), CAM_HEIGHT)) ## show twice as big 
        cv2.imshow('RealSense', img)
        # cv2.resizeWindow('RealSense', 2000, 2000)
        cv2.waitKey(1)
            # sleep_counter += 0.005
            # time.sleep(max(0, sleep_counter - time.time()))
