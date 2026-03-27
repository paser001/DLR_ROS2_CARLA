#!/usr/bin/env python3

import os
import time
import queue
import numpy as np
import cv2
import carla


HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

OUTPUT_DIR = "static_maps"
OUTPUT_RAW = "topdown_semantic_raw.png"
OUTPUT_CITYSCAPES = "topdown_semantic_cityscapes.png"

CAM_W = 1600
CAM_H = 1600
CAM_FOV = 90.0

# Center roughly around your parking lot
CAM_X = 285.0
CAM_Y = -212.0
CAM_Z = 40.0


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    cam_bp = bp_lib.find('sensor.camera.semantic_segmentation')
    cam_bp.set_attribute('image_size_x', str(CAM_W))
    cam_bp.set_attribute('image_size_y', str(CAM_H))
    cam_bp.set_attribute('fov', str(CAM_FOV))
    cam_bp.set_attribute('sensor_tick', '0.1')

    cam_tf = carla.Transform(
        carla.Location(x=CAM_X, y=CAM_Y, z=CAM_Z),
        carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
    )

    q = queue.Queue()
    cam = world.spawn_actor(cam_bp, cam_tf)
    cam.listen(q.put)

    try:
        image = q.get(timeout=5.0)

        # Save raw semantic labels
        raw = np.frombuffer(image.raw_data, dtype=np.uint8)
        raw = raw.reshape((image.height, image.width, 4))
        raw_bgr = raw[:, :, :3]
        cv2.imwrite(os.path.join(OUTPUT_DIR, OUTPUT_RAW), raw_bgr)

        # Save CityScapes palette visualization
        image.convert(carla.ColorConverter.CityScapesPalette)
        city = np.frombuffer(image.raw_data, dtype=np.uint8)
        city = city.reshape((image.height, image.width, 4))
        city_bgr = city[:, :, :3]
        cv2.imwrite(os.path.join(OUTPUT_DIR, OUTPUT_CITYSCAPES), city_bgr)

        print("Saved:")
        print(os.path.join(OUTPUT_DIR, OUTPUT_RAW))
        print(os.path.join(OUTPUT_DIR, OUTPUT_CITYSCAPES))

    finally:
        cam.stop()
        cam.destroy()


if __name__ == "__main__":
    main()