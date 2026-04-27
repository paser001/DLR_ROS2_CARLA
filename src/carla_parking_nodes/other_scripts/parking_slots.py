import math
from typing import Dict, List, Optional, Tuple


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

Z_OFFSET = 0.15


def build_parking_slots() -> List[Dict]:
    slots = []

    for i, (x, y, z) in enumerate(PARKING_LOCATIONS):
        row = i // 16 + 1
        col = i % 16 + 1

        #  parking orientation
        if row in [2, 4]:
            yaw = 180.0
        else:
            yaw = 0.0

        # side the aisle is on
        if row in [1, 3]:
            approach_sign_x = +1.0
        else:
            approach_sign_x = -1.0

        slots.append({
            "id": f"{row}-{col}",
            "row": row,
            "col": col,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "width": 2.8,
            "length": 5.5,
            "approach_sign_x": approach_sign_x,
        })

    return slots


def get_slot_by_id(slot_id: str) -> Optional[Dict]:
    for slot in build_parking_slots():
        if slot["id"] == slot_id:
            return slot
    return None


def get_slot_corners(slot: Dict) -> List[Tuple[float, float, float]]:
    cx = slot["x"]
    cy = slot["y"]
    cz = slot["z"]

    half_l = slot["length"] / 2.0
    half_w = slot["width"] / 2.0

    fl = (cx + half_l, cy - half_w, cz + Z_OFFSET)
    fr = (cx + half_l, cy + half_w, cz + Z_OFFSET)
    rr = (cx - half_l, cy + half_w, cz + Z_OFFSET)
    rl = (cx - half_l, cy - half_w, cz + Z_OFFSET)

    return [fl, fr, rr, rl, fl]


def make_target_pose(slot: Dict) -> Dict:
    return {
        "x": slot["x"],
        "y": slot["y"],
        "z": slot["z"],
        "yaw": slot["yaw"],
    }


def make_staging_pose(
    slot: Dict,
    ego_y: Optional[float] = None,
    aisle_offset_x: float = 5.5,
    maneuver_offset_y: float = 2.0,
) -> Dict:
    """
    Staging pose in the aisle before reversing into the slot.

    ego_y:
        Current ego y position, used to choose whether to stage above or below the slot.
        If None, default to the upper side of the slot.
    """
    cx = slot["x"]
    cy = slot["y"]
    cz = slot["z"]

    if ego_y is None:
        y_sign = +1.0
        approach_yaw = 90.0
    else:
        if ego_y >= cy:
            y_sign = -1.0
            approach_yaw = -90.0
        else:
            y_sign = +1.0
            approach_yaw = 90.0

    x_sign = slot["approach_sign_x"]

    return {
        "x": cx + x_sign * aisle_offset_x,
        "y": cy + y_sign * maneuver_offset_y,
        "z": cz,
        "yaw": approach_yaw,
    }


def make_entry_pose(slot: Dict, ego_y: Optional[float] = None, entry_offset_x: float = 9.0) -> Dict:
    """
    Optional earlier waypoint farther out in the aisle.
    Useful when spawn pose is not already aligned for staging.
    """
    staging = make_staging_pose(slot, ego_y=ego_y)
    x_sign = slot["approach_sign_x"]

    return {
        "x": slot["x"] + x_sign * entry_offset_x,
        "y": staging["y"],
        "z": slot["z"],
        "yaw": staging["yaw"],
    }


def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)