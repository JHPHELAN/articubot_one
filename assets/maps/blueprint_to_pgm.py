#!/usr/bin/env python3
# Author: Claude Opus 4.6 April 2026
"""
Convert Stormy house blueprint (walls-only JPG) to a Nav2 occupancy grid PGM.

Coordinate convention (matching existing Stormy maps):
  +x = Left on blueprint (West)
  +y = Up on blueprint (North)
  Robot dock origin (0,0) facing West (+x) = green crosshair in study

Steps:
  1. Load walls-only JPG (CMYK) → grayscale
  2. Threshold to binary: dark pixels = walls
  3. Horizontally flip so PGM +x maps to blueprint-left (West)
  4. Resize to 0.05 m/pixel target resolution
  5. Re-threshold after resize (anti-aliasing creates grays)
  6. Flood-fill exterior from edges → unknown (205)
  7. Open interior doorways (clear wall pixels at door locations)
  8. Save PGM (P5) and matching YAML
"""

import numpy as np
from PIL import Image
from collections import deque

# ── Configuration ──────────────────────────────────────────────
INPUT_IMAGE = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps/StormyHoustMapWallsOnly.jpg'
DOCK_IMAGE  = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps/StormyHouseMapXMarksDock.jpg'
OUTPUT_PGM  = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps/Stormy_blueprint.pgm'
OUTPUT_YAML = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps/Stormy_blueprint.yaml'
OUTPUT_PNG  = '/home/ubuntu/robot_ws/src/articubot_one/assets/maps/Stormy_blueprint.png'

REAL_WIDTH_FT = 81.0                       # total width of blueprint in feet
REAL_WIDTH_M  = REAL_WIDTH_FT * 0.3048     # 24.6888 m
TARGET_RES    = 0.05                       # meters per pixel in output PGM

# PGM pixel values (Nav2 trinary mode)
OCCUPIED = 0       # black  = wall
FREE     = 254     # white  = free space
UNKNOWN  = 205     # gray   = unknown / exterior

# Wall detection threshold (0-255 grayscale, lower = darker)
WALL_THRESH_SRC = 180   # source resolution: anything darker than this is wall
WALL_THRESH_OUT = 200   # output resolution after resize: re-threshold

# ── Step 1: Find dock (green cross) position from dock image ───
print("Finding dock position from green crosshair...")
dock_img = Image.open(DOCK_IMAGE).convert('RGB')
dock_arr = np.array(dock_img)
green_mask = (dock_arr[:,:,1] > 150) & (dock_arr[:,:,0] < 150) & (dock_arr[:,:,2] < 150)
green_coords = np.argwhere(green_mask)
dock_row, dock_col = green_coords.mean(axis=0)
print(f"  Dock pixel (before flip): col={dock_col:.0f}, row={dock_row:.0f}")

# ── Step 2: Load walls-only image → grayscale ─────────────────
print("Loading walls-only image...")
src_img = Image.open(INPUT_IMAGE).convert('L')  # CMYK → grayscale
src_w, src_h = src_img.size
print(f"  Source size: {src_w} x {src_h}")

src_arr = np.array(src_img)

# ── Step 3: Threshold at source resolution ─────────────────────
print(f"Thresholding at source resolution (< {WALL_THRESH_SRC} = wall)...")
# In grayscale: 0=black, 255=white. Walls are dark.
wall_mask = src_arr < WALL_THRESH_SRC  # True = wall

# ── Step 4: Horizontal flip (+x = left = West) ────────────────
print("Flipping horizontally (+x = West)...")
wall_mask = np.fliplr(wall_mask)
# Dock position after flip
dock_col_flipped = src_w - 1 - dock_col
print(f"  Dock pixel (after flip): col={dock_col_flipped:.0f}, row={dock_row:.0f}")

# ── Step 5: Resize to target resolution ────────────────────────
source_scale = REAL_WIDTH_M / src_w   # m/px at source
target_w = int(round(REAL_WIDTH_M / TARGET_RES))
target_h = int(round(src_h * (target_w / src_w)))
print(f"  Source scale: {source_scale*1000:.2f} mm/px")
print(f"  Target size: {target_w} x {target_h} at {TARGET_RES} m/px")

# Convert boolean wall mask to uint8 for resize (wall=0, free=255)
resize_img = Image.fromarray(np.where(wall_mask, 0, 255).astype(np.uint8))
resized = resize_img.resize((target_w, target_h), Image.LANCZOS)
out_arr = np.array(resized)

# Re-threshold after anti-aliased resize
out_walls = out_arr < WALL_THRESH_OUT  # True = wall pixel

# ── Step 6: Build output grid ──────────────────────────────────
print("Building output grid...")
grid = np.full((target_h, target_w), FREE, dtype=np.uint8)
grid[out_walls] = OCCUPIED

# ── Step 7: Flood-fill exterior as UNKNOWN ─────────────────────
print("Flood-filling exterior as unknown...")
visited = np.zeros_like(grid, dtype=bool)

# Seed from all edge pixels that are FREE
seeds = deque()
for c in range(target_w):
    for r in [0, target_h - 1]:
        if grid[r, c] == FREE and not visited[r, c]:
            seeds.append((r, c))
            visited[r, c] = True
for r in range(target_h):
    for c in [0, target_w - 1]:
        if grid[r, c] == FREE and not visited[r, c]:
            seeds.append((r, c))
            visited[r, c] = True

while seeds:
    r, c = seeds.popleft()
    grid[r, c] = UNKNOWN
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < target_h and 0 <= nc < target_w:
            if not visited[nr, nc] and grid[nr, nc] == FREE:
                visited[nr, nc] = True
                seeds.append((nr, nc))

print(f"  Occupied: {np.sum(grid == OCCUPIED)}, Free: {np.sum(grid == FREE)}, Unknown: {np.sum(grid == UNKNOWN)}")

# ── Step 8: Compute YAML origin ────────────────────────────────
# Dock = map (0,0) at pixel position in output image
scale_factor = target_w / src_w
dock_col_out = dock_col_flipped * scale_factor
dock_row_out = dock_row * scale_factor

# YAML origin = map coords of bottom-left pixel
# pixel (c, r) in image → map coords:
#   map_x = origin_x + c * resolution
#   map_y = origin_y + (height-1-r) * resolution
# At dock pixel: map_x=0, map_y=0
#   0 = origin_x + dock_col_out * resolution
#   0 = origin_y + (height-1-dock_row_out) * resolution
origin_x = -dock_col_out * TARGET_RES
origin_y = -(target_h - 1 - dock_row_out) * TARGET_RES

print(f"  Dock in output image: col={dock_col_out:.1f}, row={dock_row_out:.1f}")
print(f"  YAML origin: [{origin_x:.3f}, {origin_y:.3f}, 0]")

# ── Step 8b: Flip vertically for PGM ──────────────────────────
# JPG row 0 = top of image, but Nav2 PGM row 0 = top = highest y.
# The source JPG has North at top, but the grid was built with
# row 0 = top of JPG. We need to flip so the PGM bottom row
# matches the YAML origin (bottom-left corner).
grid = np.flipud(grid)
print("Flipped grid vertically for PGM output.")

# ── Step 9: Save PGM (binary P5) ──────────────────────────────
print(f"Saving PGM: {OUTPUT_PGM}")
with open(OUTPUT_PGM, 'wb') as f:
    header = f"P5\n{target_w} {target_h}\n255\n"
    f.write(header.encode('ascii'))
    f.write(grid.tobytes())

# Also save PNG for visual inspection
print(f"Saving PNG: {OUTPUT_PNG}")
Image.fromarray(grid).save(OUTPUT_PNG)

# ── Step 10: Save YAML ────────────────────────────────────────
print(f"Saving YAML: {OUTPUT_YAML}")
with open(OUTPUT_YAML, 'w') as f:
    f.write(f"image: Stormy_blueprint.pgm\n")
    f.write(f"mode: trinary\n")
    f.write(f"resolution: {TARGET_RES}\n")
    f.write(f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n")
    f.write(f"negate: 0\n")
    f.write(f"occupied_thresh: 0.65\n")
    f.write(f"free_thresh: 0.196\n")

print("\nDone! Files created:")
print(f"  {OUTPUT_PGM}")
print(f"  {OUTPUT_YAML}")
print(f"  {OUTPUT_PNG}")
print(f"\nTo launch:")
print(f"  ros2 launch articubot_one stingray.launch.py map:={OUTPUT_YAML}")
