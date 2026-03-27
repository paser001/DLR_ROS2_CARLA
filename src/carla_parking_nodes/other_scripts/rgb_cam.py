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
OUTPUT_RGB = "topdown_rgb_reference.png"

CAM_W = 1600
CAM_H = 1600
CAM_FOV = 90.0

# EXACT same camera pose as your semantic capture
CAM_X = 285.0
CAM_Y = -212.0
CAM_Z = 40.0


# Known parking slot centers from your lot
PARKING_LOCATIONS = [
    # row 1
    (298.5, -235.73, 0.3), (298.5, -232.73, 0.3), (298.5, -229.53, 0.3), (298.5, -226.43, 0.3),
    (298.5, -223.43, 0.3), (298.5, -220.23, 0.3), (298.5, -217.23, 0.3), (298.5, -214.03, 0.3),
    (298.5, -210.73, 0.3), (298.5, -207.30, 0.3), (298.5, -204.23, 0.3), (298.5, -201.03, 0.3),
    (298.5, -198.03, 0.3), (298.5, -194.90, 0.3), (298.5, -191.53, 0.3), (298.5, -188.20, 0.3),

    # row 2
    (290.9, -235.73, 0.3), (290.9, -232.73, 0.3), (290.9, -229.53, 0.3), (290.9, -226.43, 0.3),
    (290.9, -223.43, 0.3), (290.9, -220.23, 0.3), (290.9, -217.23, 0.3), (290.9, -214.03, 0.3),
    (290.9, -210.73, 0.3), (290.9, -207.30, 0.3), (290.9, -204.23, 0.3), (290.9, -201.03, 0.3),
    (290.9, -198.03, 0.3), (290.9, -194.90, 0.3), (290.9, -191.53, 0.3), (290.9, -188.20, 0.3),

    # row 3
    (280.0, -235.73, 0.3), (280.0, -232.73, 0.3), (280.0, -229.53, 0.3), (280.0, -226.43, 0.3),
    (280.0, -223.43, 0.3), (280.0, -220.23, 0.3), (280.0, -217.23, 0.3), (280.0, -214.03, 0.3),
    (280.0, -210.73, 0.3), (280.0, -207.30, 0.3), (280.0, -204.23, 0.3), (280.0, -201.03, 0.3),
    (280.0, -198.03, 0.3), (280.0, -194.90, 0.3), (280.0, -191.53, 0.3), (280.0, -188.20, 0.3),

    # row 4
    (272.5, -235.73, 0.3), (272.5, -232.73, 0.3), (272.5, -229.53, 0.3), (272.5, -226.43, 0.3),
    (272.5, -223.43, 0.3), (272.5, -220.23, 0.3), (272.5, -217.23, 0.3), (272.5, -214.03, 0.3),
    (272.5, -210.73, 0.3), (272.5, -207.30, 0.3), (272.5, -204.23, 0.3), (272.5, -201.03, 0.3),
    (272.5, -198.03, 0.3), (272.5, -194.90, 0.3), (272.5, -191.53, 0.3), (272.5, -188.20, 0.3),
]


def slot_center(row: int, col: int):
    idx = (row - 1) * 16 + (col - 1)
    x, y, z = PARKING_LOCATIONS[idx]
    return carla.Location(x=x, y=y, z=z)


def draw_reference_point(
    world,
    loc: carla.Location,
    label: str,
    point_color: carla.Color,
    text_color: carla.Color,
    life_time: float = 30.0
):
    point_loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.5)
    text_loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + 1.2)

    world.debug.draw_point(
        point_loc,
        size=0.18,
        color=point_color,
        life_time=life_time
    )

    world.debug.draw_string(
        text_loc,
        label,
        draw_shadow=False,
        color=text_color,
        life_time=life_time
    )


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # RGB camera with same pose as semantic camera
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(CAM_W))
    cam_bp.set_attribute('image_size_y', str(CAM_H))
    cam_bp.set_attribute('fov', str(CAM_FOV))
    cam_bp.set_attribute('sensor_tick', '0.1')

    cam_tf = carla.Transform(
        carla.Location(x=CAM_X, y=CAM_Y, z=CAM_Z),
        carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
    )

    # Draw reference points
    refs = [
        ("P11", slot_center(1, 1), carla.Color(255, 0, 0)),
        ("P116", slot_center(1, 16), carla.Color(0, 255, 0)),
        ("P41", slot_center(4, 1), carla.Color(0, 0, 255)),
        ("P416", slot_center(4, 16), carla.Color(255, 255, 0)),
        # extra interior checks
        ("P211", slot_center(2, 11), carla.Color(255, 0, 255)),
        ("P311", slot_center(3, 11), carla.Color(0, 255, 255)),
    ]

    for label, loc, color in refs:
        draw_reference_point(
            world,
            loc,
            label,
            point_color=color,
            text_color=carla.Color(255, 255, 255),
            life_time=30.0
        )

    q = queue.Queue()
    cam = world.spawn_actor(cam_bp, cam_tf)
    cam.listen(q.put)

    try:
        # small delay so debug text/points are fully visible
        time.sleep(0.5)

        image = q.get(timeout=5.0)

        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        rgb_bgr = arr[:, :, :3]

        out_path = os.path.join(OUTPUT_DIR, OUTPUT_RGB)
        cv2.imwrite(out_path, rgb_bgr)

        print("Saved:")
        print(out_path)
        print("\nReference world points used:")
        for label, loc, _ in refs:
            print(f"{label}: x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}")

    finally:
        cam.stop()
        cam.destroy()


if __name__ == "__main__":
    main()