#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np

from reeds_shepp_path_planning import reeds_shepp_path_planning


def plot_arrow(x, y, yaw, color="r", label=None):
    plt.arrow(
        x, y,
        1.0 * math.cos(yaw),
        1.0 * math.sin(yaw),
        head_width=0.3,
        head_length=0.4,
        fc=color,
        ec=color,
        label=label
    )


def main():
    # Example CARLA start pose
    sx = 280.45
    sy = -204.20
    syaw = math.radians(70.0)

    # Cargo box goal pose
    gx = 290.9
    gy = -201.03
    gyaw = math.radians(180.0)

    min_turning_radius = 6.0
    maxc = 1.0 / min_turning_radius
    step_size = 0.2

    xs, ys, yaws, modes, lengths, directions = reeds_shepp_path_planning(
        sx, sy, syaw,
        gx, gy, gyaw,
        maxc,
        step_size
    )

    if xs is None:
        print("No Reeds-Shepp path found")
        return

    print("modes:", modes)
    print("lengths:", lengths)
    print("directions:", directions[:20], "...")

    xs = np.array(xs)
    ys = np.array(ys)
    directions = np.array(directions)

    forward = directions > 0
    reverse = directions < 0

    plt.figure()
    plt.plot(xs[forward], ys[forward], ".b", label="forward")
    plt.plot(xs[reverse], ys[reverse], ".r", label="reverse")

    plot_arrow(sx, sy, syaw, color="g", label="start")
    plot_arrow(gx, gy, gyaw, color="k", label="goal")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Reeds-Shepp docking path")
    plt.show()


if __name__ == "__main__":
    main()