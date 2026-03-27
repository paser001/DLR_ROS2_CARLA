import os
import cv2
import numpy as np

os.makedirs("static_maps/final", exist_ok=True)

raw = cv2.imread("static_maps/topdown_semantic_raw.png", cv2.IMREAD_UNCHANGED)
labels = raw[:, :, 2]

print("Unique labels:", np.unique(labels))

# 1 = occupied, 0 = free
grid = np.ones(labels.shape, dtype=np.uint8)

# Parking lot surface + markings
free_mask = (labels == 25) | (labels == 24)
grid[free_mask] = 0

np.save("static_maps/final/parking_occupancy.npy", grid)

vis = (1 - grid) * 255
cv2.imwrite("static_maps/final/parking_occupancy.png", vis)

print("Saved:")
print("  static_maps/final/parking_occupancy.npy")
print("  static_maps/final/parking_occupancy.png")