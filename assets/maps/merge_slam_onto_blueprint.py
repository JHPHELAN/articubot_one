#!/usr/bin/env python3
"""
Merge SLAM-detected obstacles onto the blueprint map (or a previous merged map).

Takes a base PGM (blueprint or previous merge) and overlays
occupied pixels from a SLAM-generated PGM (furniture, obstacles).

The two maps have different origins and sizes but the same resolution (0.05 m/px).
We align them using their YAML origins and copy SLAM occupied pixels
onto the base where the base currently shows free space.

Usage:
  python3 merge_slam_onto_blueprint.py <slam_map_name> [base_map_name]
  python3 merge_slam_onto_blueprint.py Stormy_v08              # merge onto blueprint
  python3 merge_slam_onto_blueprint.py Stormy_v09 Stormy_merged_v08  # merge onto previous
"""

import sys
import yaml
import numpy as np
from PIL import Image

# ── Parse arguments ────────────────────────────────────────────
if len(sys.argv) < 2 or len(sys.argv) > 3:
    print(f"Usage: {sys.argv[0]} <slam_map_name> [base_map_name]")
    print(f"  e.g.: {sys.argv[0]} Stormy_v08")
    print(f"  e.g.: {sys.argv[0]} Stormy_v09 Stormy_merged_v08")
    sys.exit(1)

slam_name = sys.argv[1]
base_name = sys.argv[2] if len(sys.argv) == 3 else None
MAPS_DIR = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps'

# ── Configuration ──────────────────────────────────────────────
if base_name:
    BLUEPRINT_PGM  = f'{MAPS_DIR}/{base_name}.pgm'
    BLUEPRINT_YAML = f'{MAPS_DIR}/{base_name}.yaml'
else:
    BLUEPRINT_PGM  = f'{MAPS_DIR}/Stormy_blueprint.pgm'
    BLUEPRINT_YAML = f'{MAPS_DIR}/Stormy_blueprint.yaml'
SLAM_PGM       = f'{MAPS_DIR}/{slam_name}.pgm'
SLAM_YAML      = f'{MAPS_DIR}/{slam_name}.yaml'
OUTPUT_PGM     = f'{MAPS_DIR}/Stormy_merged.pgm'
OUTPUT_PNG     = f'{MAPS_DIR}/Stormy_merged.png'
OUTPUT_YAML    = f'{MAPS_DIR}/Stormy_merged.yaml'

# Read origins from YAML files
def read_origin(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    origin = data['origin']
    return (origin[0], origin[1])

BLUEPRINT_ORIGIN = read_origin(BLUEPRINT_YAML)
SLAM_ORIGIN      = read_origin(SLAM_YAML)
RESOLUTION = 0.05  # both maps

base_label = base_name if base_name else 'blueprint'
print(f"Merging {slam_name} onto {base_label}...")
print(f"  Base origin: {BLUEPRINT_ORIGIN}")
print(f"  SLAM origin: {SLAM_ORIGIN}")

# PGM values
OCCUPIED = 0
FREE     = 254
UNKNOWN  = 205

# Threshold for "occupied" in SLAM PGM (SLAM uses 0=occupied, 254=free, 205=unknown)
SLAM_OCC_THRESH = 50  # anything below this in SLAM map = obstacle

# ── Load maps ──────────────────────────────────────────────────
print("Loading blueprint...")
bp_img = Image.open(BLUEPRINT_PGM)
bp = np.array(bp_img)
bp_h, bp_w = bp.shape
print(f"  Blueprint: {bp_w} x {bp_h}")

print("Loading SLAM map...")
slam_img = Image.open(SLAM_PGM)
slam = np.array(slam_img)
slam_h, slam_w = slam.shape
print(f"  SLAM: {slam_w} x {slam_h}")

# ── Compute pixel offset between the two maps ─────────────────
# Both maps use the same resolution, so we just need the pixel offset.
# For a PGM: pixel (col, row) corresponds to map coords:
#   map_x = origin_x + col * resolution
#   map_y = origin_y + (height - 1 - row) * resolution
#
# To find where SLAM pixel (sc, sr) lands on the blueprint:
#   map_x = slam_origin_x + sc * res
#   map_y = slam_origin_y + (slam_h - 1 - sr) * res
#   bp_col = (map_x - bp_origin_x) / res
#   bp_row = (bp_h - 1) - (map_y - bp_origin_y) / res

# Offset in pixels: where SLAM (0,0) pixel maps to in blueprint pixel space
# SLAM pixel (0, slam_h-1) = bottom-left = SLAM origin in map coords
# That maps to blueprint pixel:
dx_m = SLAM_ORIGIN[0] - BLUEPRINT_ORIGIN[0]  # x offset in meters
dy_m = SLAM_ORIGIN[1] - BLUEPRINT_ORIGIN[1]  # y offset in meters

col_offset = int(round(dx_m / RESOLUTION))  # SLAM col 0 → blueprint col col_offset
row_offset = int(round(dy_m / RESOLUTION))  # y offset in pixels

print(f"  Offset: dx={dx_m:.3f}m, dy={dy_m:.3f}m")
print(f"  Pixel offset: col={col_offset}, row_y={row_offset}")

# ── Copy SLAM obstacles onto blueprint ─────────────────────────
print("Merging SLAM obstacles onto blueprint...")
merged = bp.copy()
obstacles_added = 0

for sr in range(slam_h):
    for sc in range(slam_w):
        if slam[sr, sc] >= SLAM_OCC_THRESH:
            continue  # not an obstacle in SLAM

        # Convert SLAM pixel to map coords
        map_x = SLAM_ORIGIN[0] + sc * RESOLUTION
        map_y = SLAM_ORIGIN[1] + (slam_h - 1 - sr) * RESOLUTION

        # Convert map coords to blueprint pixel
        # Standard Nav2 PGM formula (same as SLAM)
        bp_col = int(round((map_x - BLUEPRINT_ORIGIN[0]) / RESOLUTION))
        bp_row = int(round((bp_h - 1) - (map_y - BLUEPRINT_ORIGIN[1]) / RESOLUTION))

        # Check bounds
        if 0 <= bp_col < bp_w and 0 <= bp_row < bp_h:
            # Only add obstacle where blueprint shows FREE space
            # (don't overwrite existing walls or unknown)
            if merged[bp_row, bp_col] == FREE:
                merged[bp_row, bp_col] = OCCUPIED
                obstacles_added += 1

print(f"  Obstacles added: {obstacles_added}")
print(f"  Blueprint occupied: {np.sum(bp == OCCUPIED)} → Merged occupied: {np.sum(merged == OCCUPIED)}")

# ── Save output ────────────────────────────────────────────────
print(f"Saving PGM: {OUTPUT_PGM}")
with open(OUTPUT_PGM, 'wb') as f:
    header = f"P5\n{bp_w} {bp_h}\n255\n"
    f.write(header.encode('ascii'))
    f.write(merged.tobytes())

print(f"Saving PNG: {OUTPUT_PNG}")
Image.fromarray(merged).save(OUTPUT_PNG)

print(f"Saving YAML: {OUTPUT_YAML}")
with open(OUTPUT_YAML, 'w') as f:
    f.write(f"image: Stormy_merged.pgm\n")
    f.write(f"mode: trinary\n")
    f.write(f"resolution: {RESOLUTION}\n")
    f.write(f"origin: [{BLUEPRINT_ORIGIN[0]:.3f}, {BLUEPRINT_ORIGIN[1]:.3f}, 0]\n")
    f.write(f"negate: 0\n")
    f.write(f"occupied_thresh: 0.65\n")
    f.write(f"free_thresh: 0.196\n")

print(f"\nDone! Merged map uses blueprint origin: {BLUEPRINT_ORIGIN}")
print(f"To launch with AMCL:")
print(f"  ros2 launch articubot_one stingray.launch.py localizer_type:=amcl map:={OUTPUT_YAML}")
