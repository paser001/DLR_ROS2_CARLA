#!/usr/bin/env python3

import json
import math
import os

import carla
import cv2
import numpy as np


HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# Parking-lot region around your Town04 parking area
X_MIN = 265.0
X_MAX = 305.0
Y_MIN = -245.0
Y_MAX = -180.0

RESOLUTION = 0.20  # meters per cell

WAYPOINT_SPACING = 0.5
LANE_SAMPLES_ACROSS = 5

OUTPUT_DIR = "static_maps"
GRID_NPY = "town04_parking_grid.npy"
GRID_PNG = "town04_parking_grid.png"
META_JSON = "town04_parking_meta.json"


def world_to_grid(x, y, x_min, y_min, resolution):
    gx = int((x - x_min) / resolution)
    gy = int((y - y_min) / resolution)
    return gx, gy


def in_bounds(gx, gy, width, height):
    return 0 <= gx < width and 0 <= gy < height


def draw_disk(grid, gx, gy, radius_cells, value):
    h, w = grid.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                x = gx + dx
                y = gy + dy
                if 0 <= x < w and 0 <= y < h:
                    grid[y, x] = value


def get_actor_bbox_polygon(actor: carla.Actor):
    tf = actor.get_transform()
    bb = actor.bounding_box

    yaw = math.radians(tf.rotation.yaw)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    cx = tf.location.x + bb.location.x * cos_y - bb.location.y * sin_y
    cy = tf.location.y + bb.location.x * sin_y + bb.location.y * cos_y

    ex = bb.extent.x
    ey = bb.extent.y

    corners_local = [
        (+ex, +ey),
        (+ex, -ey),
        (-ex, -ey),
        (-ex, +ey),
    ]

    corners_world = []
    for lx, ly in corners_local:
        wx = cx + lx * cos_y - ly * sin_y
        wy = cy + lx * sin_y + ly * cos_y
        corners_world.append((wx, wy))

    return corners_world


def polygon_to_grid_points(poly_world, x_min, y_min, resolution):
    pts = []
    for x, y in poly_world:
        gx, gy = world_to_grid(x, y, x_min, y_min, resolution)
        pts.append([gx, gy])
    return np.array(pts, dtype=np.int32)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    world = client.get_world()
    carla_map = world.get_map()

    width = int(math.ceil((X_MAX - X_MIN) / RESOLUTION))
    height = int(math.ceil((Y_MAX - Y_MIN) / RESOLUTION))

    # 1 = occupied, 0 = free
    grid = np.ones((height, width), dtype=np.uint8)

    print("Generating waypoints...")
    waypoints = carla_map.generate_waypoints(WAYPOINT_SPACING)
    print(f"Total waypoints: {len(waypoints)}")

    # Mark drivable space from waypoints
    for wp in waypoints:
        loc = wp.transform.location

        if not (X_MIN <= loc.x <= X_MAX and Y_MIN <= loc.y <= Y_MAX):
            continue

        lane_half_width = 0.5 * wp.lane_width

        yaw = math.radians(wp.transform.rotation.yaw)
        right_x = -math.sin(yaw)
        right_y = math.cos(yaw)

        for alpha in np.linspace(-1.0, 1.0, LANE_SAMPLES_ACROSS):
            sx = loc.x + alpha * lane_half_width * right_x
            sy = loc.y + alpha * lane_half_width * right_y

            gx, gy = world_to_grid(sx, sy, X_MIN, Y_MIN, RESOLUTION)
            if in_bounds(gx, gy, width, height):
                draw_disk(grid, gx, gy, radius_cells=1, value=0)

    print("Marking parked vehicles as occupied...")
    vehicles = world.get_actors().filter("vehicle.*")
    print(f"Vehicles found: {len(vehicles)}")

    for actor in vehicles:
        loc = actor.get_location()
        if not (X_MIN - 5 <= loc.x <= X_MAX + 5 and Y_MIN - 5 <= loc.y <= Y_MAX + 5):
            continue

        poly_world = get_actor_bbox_polygon(actor)
        poly_grid = polygon_to_grid_points(poly_world, X_MIN, Y_MIN, RESOLUTION)

        cv2.fillPoly(grid, [poly_grid], color=1)

    # Save raw grid
    grid_path = os.path.join(OUTPUT_DIR, GRID_NPY)
    np.save(grid_path, grid)

    # Save visualization
    vis = (1 - grid) * 255  # free=white, occupied=black
    vis_path = os.path.join(OUTPUT_DIR, GRID_PNG)
    cv2.imwrite(vis_path, vis)

    # Save metadata
    meta = {
        "host": HOST,
        "port": PORT,
        "x_min": X_MIN,
        "x_max": X_MAX,
        "y_min": Y_MIN,
        "y_max": Y_MAX,
        "resolution": RESOLUTION,
        "width": width,
        "height": height,
        "waypoint_spacing": WAYPOINT_SPACING,
    }
    meta_path = os.path.join(OUTPUT_DIR, META_JSON)
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    print("Saved:")
    print(f"  {grid_path}")
    print(f"  {vis_path}")
    print(f"  {meta_path}")


if __name__ == "__main__":
    main()