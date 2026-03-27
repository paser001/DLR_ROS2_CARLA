#!/usr/bin/env python3

import os
import cv2
import numpy as np

# -------------------------------------------------------------------
# Paths
# -------------------------------------------------------------------
IMG_PATH = "static_maps/topdown_rgb_reference.png"
OUTPUT_DIR = "static_maps/final"

# -------------------------------------------------------------------
# Display scale for easier viewing on screen
# -------------------------------------------------------------------
DISPLAY_SCALE = 0.6

# -------------------------------------------------------------------
# Known world points used in the RGB reference image
# IMPORTANT: click them in this exact order
# -------------------------------------------------------------------
WORLD_POINTS = np.array([
    [298.5, -235.73],   # P11
    [298.5, -188.20],   # P116
    [272.5, -235.73],   # P41
    [272.5, -188.20],   # P416
], dtype=np.float32)

WORLD_LABELS = ["P11", "P116", "P41", "P416"]

# Optional extra points for validation after fitting
CHECK_WORLD_POINTS = np.array([
    [290.9, -204.23],   # P211
    [280.0, -204.23],   # P311
], dtype=np.float32)

CHECK_WORLD_LABELS = ["P211", "P311"]

clicked_points = []
img_vis = None


def world_to_image(x, y, H_world_to_img):
    p = np.array([x, y, 1.0], dtype=np.float64)
    q = H_world_to_img @ p
    q /= q[2]
    return float(q[0]), float(q[1])


def image_to_world(u, v, H_img_to_world):
    p = np.array([u, v, 1.0], dtype=np.float64)
    q = H_img_to_world @ p
    q /= q[2]
    return float(q[0]), float(q[1])


def redraw_image(base_img):
    global img_vis
    img_vis = base_img.copy()

    # Draw already-clicked points on the scaled display image
    for i, (orig_x, orig_y) in enumerate(clicked_points):
        disp_x = int(round(orig_x * DISPLAY_SCALE))
        disp_y = int(round(orig_y * DISPLAY_SCALE))

        cv2.circle(img_vis, (disp_x, disp_y), 6, (0, 0, 255), -1)
        cv2.putText(
            img_vis,
            str(i + 1),
            (disp_x + 8, disp_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )

    # Show next point to click
    next_idx = len(clicked_points)
    if next_idx < len(WORLD_LABELS):
        msg = f"Click next: {WORLD_LABELS[next_idx]}"
    else:
        msg = "Press ENTER to fit homography"

    cv2.putText(
        img_vis,
        msg,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )


def mouse_callback(event, x, y, flags, param):
    global clicked_points, img_vis
    base_img = param

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_points) < len(WORLD_POINTS):
            # Convert click from scaled display image to original image coordinates
            orig_x = int(round(x / DISPLAY_SCALE))
            orig_y = int(round(y / DISPLAY_SCALE))

            clicked_points.append([orig_x, orig_y])
            print(
                f"Clicked {len(clicked_points)}: "
                f"display=({x}, {y}) -> original=({orig_x}, {orig_y}) "
                f"for {WORLD_LABELS[len(clicked_points) - 1]}"
            )

            redraw_image(base_img)
            cv2.imshow("reference_image", img_vis)

    elif event == cv2.EVENT_RBUTTONDOWN:
        if len(clicked_points) > 0:
            removed = clicked_points.pop()
            print(f"Removed last point: {removed}")
            redraw_image(base_img)
            cv2.imshow("reference_image", img_vis)


def main():
    global img_vis

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    img = cv2.imread(IMG_PATH)
    if img is None:
        raise RuntimeError(f"Could not load image: {IMG_PATH}")

    img_display = cv2.resize(
        img,
        None,
        fx=DISPLAY_SCALE,
        fy=DISPLAY_SCALE,
        interpolation=cv2.INTER_AREA
    )

    print("Click these world points in this exact order:")
    for i, (label, pt) in enumerate(zip(WORLD_LABELS, WORLD_POINTS), start=1):
        print(f"{i}. {label}: world=({pt[0]}, {pt[1]})")

    print("\nControls:")
    print("  Left click  -> add point")
    print("  Right click -> remove last point")
    print("  Enter       -> fit homography after 4 clicks")
    print("  Esc         -> quit")

    redraw_image(img_display)
    cv2.imshow("reference_image", img_vis)
    cv2.setMouseCallback("reference_image", mouse_callback, img_display)

    while True:
        key = cv2.waitKey(20) & 0xFF

        if key == 27:  # ESC
            print("Aborted.")
            cv2.destroyAllWindows()
            return

        elif key == 13:  # ENTER
            if len(clicked_points) != len(WORLD_POINTS):
                print(f"Need {len(WORLD_POINTS)} points, currently have {len(clicked_points)}")
                continue
            break

    cv2.destroyAllWindows()

    image_points = np.array(clicked_points, dtype=np.float32)

    H_world_to_img, mask = cv2.findHomography(WORLD_POINTS, image_points)
    if H_world_to_img is None:
        raise RuntimeError("Failed to compute homography")

    H_img_to_world = np.linalg.inv(H_world_to_img)

    np.save(os.path.join(OUTPUT_DIR, "H_world_to_img.npy"), H_world_to_img)
    np.save(os.path.join(OUTPUT_DIR, "H_img_to_world.npy"), H_img_to_world)
    np.save(os.path.join(OUTPUT_DIR, "clicked_image_points.npy"), image_points)
    np.save(os.path.join(OUTPUT_DIR, "world_points.npy"), WORLD_POINTS)

    print("\nSaved:")
    print(os.path.join(OUTPUT_DIR, "H_world_to_img.npy"))
    print(os.path.join(OUTPUT_DIR, "H_img_to_world.npy"))
    print(os.path.join(OUTPUT_DIR, "clicked_image_points.npy"))
    print(os.path.join(OUTPUT_DIR, "world_points.npy"))

    print("\nH_world_to_img =")
    print(H_world_to_img)

    print("\nH_img_to_world =")
    print(H_img_to_world)

    # -------------------------------------------------------------
    # Verification image
    # -------------------------------------------------------------
    check_img = img.copy()

    # Draw clicked calibration points in red
    for i, (u, v) in enumerate(image_points):
        u_i = int(round(u))
        v_i = int(round(v))
        cv2.circle(check_img, (u_i, v_i), 7, (0, 0, 255), -1)
        cv2.putText(
            check_img,
            WORLD_LABELS[i],
            (u_i + 8, v_i - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )

    # Draw extra validation points in green
    for label, (x, y) in zip(CHECK_WORLD_LABELS, CHECK_WORLD_POINTS):
        u, v = world_to_image(x, y, H_world_to_img)
        u_i = int(round(u))
        v_i = int(round(v))

        cv2.circle(check_img, (u_i, v_i), 7, (0, 255, 0), -1)
        cv2.putText(
            check_img,
            label,
            (u_i + 8, v_i - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        print(f"{label}: world=({x:.2f}, {y:.2f}) -> image=({u:.2f}, {v:.2f})")

    verify_path = os.path.join(OUTPUT_DIR, "homography_check.png")
    cv2.imwrite(verify_path, check_img)
    print(f"\nSaved verification image: {verify_path}")


if __name__ == "__main__":
    main()