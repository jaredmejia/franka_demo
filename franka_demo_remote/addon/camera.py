import time
import numpy as np
import cv2
import pyrealsense2 as rs
from PIL import Image
from threading import Thread
import queue
from multiprocessing import Queue as MPQueue
from multiprocessing import Process

from franka_demo_remote.utils import print_and_cr
from franka_demo_remote.remote_utils import redis_send_frame

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30
CAM_KEYS = ["cam0c", "cam1c", "cam2c"]
FRAME_TYPE = np.uint8
NUM_WRITERS = 8

def add_camera_function(state):
    state.is_logging_to = None
    state.cam_recorder_queue = MPQueue()
    state.cam_recorder_proc = Process(
        target=record_camera,
        args=(state.cam_recorder_queue, NUM_WRITERS)).start()

    state.cameras = RealSense(state)
    state.onclose.append(close_cameras)
    state.handlers['C'] = debug_update_camera_fps              # FOR DEBUG
    state.handlers['D'] = debug_camera_connection               # FOR DEBUG

class RealSense:
    """ Wrapper that implements boilerplate code for RealSense cameras """

    def __init__(self, state):
        # TODO read initialization from a config file

        self.device_ls = []
        for cam in rs.context().query_devices():
            self.device_ls.append(cam.get_info(rs.camera_info(1)))
        self.device_ls.sort()

        desired_device_ls = ["101622070870", "815412070907", "818312070212"]
        for device in desired_device_ls:
            assert(device in self.device_ls)

        self.device_ls = desired_device_ls

        # Start streaming
        print_and_cr(f"Connecting to RealSense cameras ({len(self.device_ls)} found) ...")
        self.pipes = []
        for i, device_id in enumerate(self.device_ls):
            pipe = rs.pipeline()
            config = rs.config()

            config.enable_device(device_id)
            config.enable_stream(rs.stream.depth, CAM_WIDTH, CAM_HEIGHT, rs.format.z16, CAM_FPS)
            config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)

            pipe.start(config)
            self.pipes.append((device_id, pipe))

            print_and_cr(f"Connected to camera {i} ({device_id}).")

        print_and_cr(f"[INFO] Warm start cameras (realsense auto-adjusts brightness during initial frames)")
        for _ in range(60):
            [pipe.poll_for_frames() for _,pipe in self.pipes]
            time.sleep(0.033)

        # Keep polling for frames in a background thread
        self.cam_state = {}
        self.pull_thread = Thread(target=update_camera, name="Update cameras",
                                  args=(self.pipes, self.cam_state, state),
                                  daemon=True)
        self.pull_thread.start()

        # self.visual_thread = Thread(target=render_cam_state, name="Render camera states",
        #                           args=[state])
        # self.visual_thread.start()

        print_and_cr(f"[INFO] Camera setup completed.")

    def get_num_cameras(self):
        return len(self.device_ls)

    def get_data(self):
        return self.cam_state

    #def start_logging(self, state):
    #    assert state.is_logging_to is not None
    #    state.cam_recorder_queue.put(state.is_logging_to)

    #def terminate_logging(self, state):
    #    state.cam_recorder_queue.put(None)
    #    state.is_logging_to = None


    #def close_logger(self, state):
    #    #print(f"[DEBUG] Closing camera logger")
    #    state.cam_recorder_queue.put(-1)
    #    self.cam_logger.join()
    #    #print(f"[DEBUG] Camera Logger closed.")

def update_camera(pipes, cam_state, state):
    """ Update camera info"""
    align_to = rs.stream.color
    align = rs.align(align_to)
    print_and_cr("[INFO] Streaming cameras")

    # FOR DEBUG
    state.cam_counter = {}
    for device_id, pipe in pipes:                   # FOR DEBUG
        state.cam_counter[device_id] = []           # FOR DEBUG

    sleep_counter = time.time()
    while not state.quit:
        for i, (device_id, pipe) in enumerate(pipes):
            frames = pipe.poll_for_frames()
            if (frames.is_frameset()):
                align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_timestamp = depth_frame.get_timestamp()
                color_image = np.asanyarray(color_frame.get_data())
                color_timestamp = color_frame.get_timestamp()
                cam_state[f"cam{i}c"] = (color_image, color_timestamp)
                cam_state[f"cam{i}d"] = (depth_image, depth_timestamp)
                state.cam_counter[device_id].append(time.time()) # FOR DEBUG
                if state.is_logging_to:
                    state.cam_recorder_queue.put((state.is_logging_to, i, device_id, color_image, color_timestamp, depth_image, depth_timestamp))
        if state.params['remote'] and all([CAM_KEYS[i] in cam_state.keys() for i in range(len(CAM_KEYS))]):
            redis_send_frame(state.redis_store, cam_state)
        sleep_counter += 0.005
        time.sleep(max(0, sleep_counter - time.time()))
        
#### moved to a separate file ####
# def redis_send_frame(redis_store, cam_state):
#     for key in CAM_KEYS:
#         redis_store.set(key, cam_state[key][0].tobytes())

# def redis_receive_frame(redis_store):
#     cam_state = {}
#     for key in CAM_KEYS:
#         cam_state[key] = np.array(np.frombuffer(redis_store.get(key), dtype=FRAME_TYPE).reshape([CAM_HEIGHT, CAM_WIDTH, 3]))
#     return cam_state

# def render_cam_state(state):
#     """ Update camera info"""
#     print("rendering in progress.......")
#     while not state.quit:
#         cam_state = redis_receive_frame(state.redis_store)
#         imgs_ti = []
#         for i in range(len(CAM_KEYS)):
#             color_image = cam_state[CAM_KEYS[i]]
#             imgs_ti.append(color_image)
#         stacked_imgs_ti = np.hstack(imgs_ti)
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         # img = cv2.resize(stacked_imgs_ti, (CAM_WIDTH * len(CAM_KEYS) * 2, CAM_HEIGHT * 2)) ## show twice as big 
#         img = cv2.resize(stacked_imgs_ti, (CAM_WIDTH * len(CAM_KEYS), CAM_HEIGHT)) ## show twice as big 
#         cv2.imshow('RealSense', img)
#         # cv2.resizeWindow('RealSense', 2000, 2000)
#         cv2.waitKey(1)
#             # sleep_counter += 0.005
#             # time.sleep(max(0, sleep_counter - time.time()))

def camera_writer(q):
    while True:
        fn, image = q.get()
        if isinstance(fn, int):
            break
        image.save(fn)

def record_camera(proc_queue, num_writers):
    """ Process the camera data and use threads to write to disk"""
    writer_queue = queue.Queue()
    my_threads = []
    for _ in range(num_writers):
        t = Thread(target=camera_writer, args=(writer_queue,))
        t.start()
        my_threads.append(t)

    while True:
        item = proc_queue.get()
        if item is None:
            print_and_cr(f"Quitting camera recording")
            break

        fn_prefix, idx, device_id, color_image, color_timestamp, depth_image, depth_timestamp = item
        color_im = Image.fromarray(color_image[:,:,::-1])
        writer_queue.put((
            f"{fn_prefix}/c{idx}-{device_id}-{color_timestamp}-color.jpeg",
            color_im
        ))
        depth_im = Image.fromarray(depth_image.astype(np.uint8))
        writer_queue.put((
            f"{fn_prefix}/c{idx}-{device_id}-{depth_timestamp}-depth.jpeg",
            depth_im
        ))

    for _ in range(num_writers):
        writer_queue.put((-1, -1))
    for thread in my_threads:
        thread.join()


def debug_update_camera_fps(key_pressed, state):
    """ Print the FPS for each camera"""
    for device_id in state.cameras.device_ls:
        counter = state.cam_counter[device_id]
        if len(counter) == 0:
            print_and_cr(f"{device_id} didn't receive any update")
        elif time.time() - counter[-1] > 0.5:
            print_and_cr(f"{device_id} didn't received update in {time.time() - counter[-1]:.1f} sec")
            continue
        else:
            if len(counter) > 100:
                counter = counter[-100:]
            print_and_cr(f"{device_id}: {len(counter) / (counter[-1] - counter[0])} FPS")


def list_realsense():
    try:
        from itertools import chain
        import usb.core
        devs = chain(usb.core.find(idVendor=0x8086, idProduct=0x0b07, find_all=True),
                    usb.core.find(idVendor=0x8086, idProduct=0x0b3a, find_all=True))
        for dev in devs:
            print_and_cr(f"Found {dev.product} " +
                f"bus {dev.bus} port {dev.port_number}")
    except e:
        print(e)

def debug_camera_connection(key_pressed, state):
    list_realsense()

def close_cameras(state):
    state.cam_recorder_queue.put(None)
    state.cam_recorder_queue.close()
    state.cam_recorder_queue.join_thread()
    state.cameras.pull_thread.join() # (Optional for daemon=True threads)


if __name__ == "__main__":
    pass