#!/usr/bin/env python3

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


Cell = Tuple[int, int]


@dataclass
class FrontierCandidate:
    x: float
    y: float
    cluster_size: int
    distance_to_robot: float
    score: float


class FrontierExplorerV2(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer_v2")

        # Safety-first default: evaluate and log candidates only.
        self.declare_parameter("dry_run", True)
        self.declare_parameter("loop_hz", 0.33)  # about every 3 seconds
        # 2026-05-17: Tried min_cluster_size=15 to suppress phantom-gap goals; diag showed
        # 174/177 clusters rejected on a fresh map (avg ~1.9 cells/cluster early on), so 15
        # starves the explorer. Reverted to 6. Phantom-gap suppression now lives in the
        # clearance check (min_clearance_m) below, which is the actual geometric signature
        # of a wormhole through a wall.
        self.declare_parameter("min_cluster_size", 6)
        self.declare_parameter("min_goal_distance_m", 0.6)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("recent_goal_memory", 6)
        self.declare_parameter("failures_before_blacklist", 2)
        self.declare_parameter("goal_key_resolution_m", 0.2)
        self.declare_parameter("no_path_recovery_cycles", 6)
        self.declare_parameter("costmap_clear_cooldown_sec", 30.0)
        # 2026-05-17: Tight-passage avoidance. Reject frontier goals whose centroid is closer
        # than min_clearance_m to any occupied cell in /map. Robot half-width is ~0.14 m, so
        # 0.40 m leaves ~26 cm clearance per side at the goal. Tune up to stay further out
        # of tight alleys, down to be more adventurous. (Tried 0.45 to filter phantom gaps —
        # starved startup. Tried 0.30 to unblock — too permissive. Back to 0.40 as the
        # documented sweet spot.)
        self.declare_parameter("min_clearance_m", 0.40)
        self.declare_parameter("clearance_score_weight", 4.0)
        # 2026-05-17: Nearer-first preference. Cap the cluster bonus so a giant remote
        # frontier can't overwhelm a modest nearby one, and apply a stronger distance
        # penalty. "Nearest open nook before farthest field."
        self.declare_parameter("cluster_score_cap", 30.0)
        self.declare_parameter("distance_score_weight", 2.5)
        # 2026-05-17 (PM): "Local planner says I ain't goin that way" filter. After the
        # global planner returns a path, compare its length to the straight-line distance.
        # If the ratio exceeds max_path_detour_ratio, the goal requires a crazy detour
        # (e.g. routing around the whole house through chair-leg gauntlets to reach a
        # frontier on the other side of a wall) — reject and pick a different candidate.
        # 1.0 = straight line, 1.5 = mild dogleg, 3.0 = clearly going the long way around.
        # Set to 0 to disable.
        # Tuning history (2026-05-17 evening): 2.5 didn't catch enough back-door routes,
        # tried 2.0 which rejected legitimate cross-room hallway goals (straight line
        # cuts through walls, real path winds through doorways = 2-3x naturally), settled
        # on 3.0 which still rejects 4-7x nonsense we've actually seen in logs.
        self.declare_parameter("max_path_detour_ratio", 3.0)
        # 2026-05-17 (PM): When a goal hits failures_before_blacklist, blacklist a radius
        # around it (not just the 0.2 m goal_key cell). Prevents the explorer from picking
        # nearby variants of the same impossible passage (e.g. 17.48,3.35 then 17.78,3.02).
        # 0.8 m is roughly one robot length — "don't try anything within a robot-length
        # of a known-bad spot."
        self.declare_parameter("blacklist_radius_m", 0.8)
        # 2026-05-17 (PM): "Get your nose out of there" filter. After the lidar has seen the
        # boundaries of a small pocket (under step-stool, between chair legs, etc.), tiny
        # slivers of unknown remain visible through gaps and look like frontiers. If the
        # connected unknown region behind a cluster is smaller than min_unknown_pocket_cells,
        # the pocket is already characterized — skip it. 25 cells ≈ 0.0625 m² at 0.05 m/cell
        # (roughly a 25 cm × 25 cm square). Set to 0 to disable.
        self.declare_parameter("min_unknown_pocket_cells", 25)        # 2026-05-18: Path-corridor clearance filter. After the global planner returns a
        # path, sample it against the inflated global costmap. If any sampled cell along
        # the path (past path_clearance_skip_start_m, to ignore the robot's own footprint
        # at the start) has cost >= path_clearance_max_cost on the 0-100 OccupancyGrid
        # scale, reject the candidate as a corridor squeeze (e.g. threading a 30 cm gap
        # between chair legs to a pocket of unknown behind furniture). 90 sits just below
        # inscribed (99) — catches the squeeze zone without rejecting normal doorways.
        # Set path_clearance_check to false to disable.
        self.declare_parameter("path_clearance_check", True)
        self.declare_parameter("path_clearance_max_cost", 90)
        self.declare_parameter("path_clearance_sample_step_m", 0.10)
        self.declare_parameter("path_clearance_skip_start_m", 0.35)
        # 2026-05-30: Goal-cell cost pre-check. Before scoring a candidate, look up
        # the global costmap value at the candidate's centroid. If cost is at/above
        # goal_max_cost (0-100 scale), the goal sits inside the inflation halo of an
        # obstacle and the planner will abort with status 6. Reject early to avoid
        # 24x getPath aborts per cycle. Reuses path_clearance_max_cost by default
        # (90 = just below inscribed, leaves doorway slack).
        self.declare_parameter("goal_cost_check", True)
        self.declare_parameter("goal_max_cost", 90)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        loop_hz = float(self.get_parameter("loop_hz").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance_m").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.recent_goal_memory = int(self.get_parameter("recent_goal_memory").value)
        self.failures_before_blacklist = int(self.get_parameter("failures_before_blacklist").value)
        self.goal_key_resolution = float(self.get_parameter("goal_key_resolution_m").value)
        self.no_path_recovery_cycles = int(self.get_parameter("no_path_recovery_cycles").value)
        self.costmap_clear_cooldown_sec = float(self.get_parameter("costmap_clear_cooldown_sec").value)
        self.min_clearance_m = float(self.get_parameter("min_clearance_m").value)
        self.clearance_score_weight = float(self.get_parameter("clearance_score_weight").value)
        self.cluster_score_cap = float(self.get_parameter("cluster_score_cap").value)
        self.distance_score_weight = float(self.get_parameter("distance_score_weight").value)
        self.max_path_detour_ratio = float(self.get_parameter("max_path_detour_ratio").value)
        self.blacklist_radius_m = float(self.get_parameter("blacklist_radius_m").value)
        self.min_unknown_pocket_cells = int(self.get_parameter("min_unknown_pocket_cells").value)
        self.path_clearance_check = bool(self.get_parameter("path_clearance_check").value)
        self.path_clearance_max_cost = int(self.get_parameter("path_clearance_max_cost").value)
        self.path_clearance_sample_step_m = float(self.get_parameter("path_clearance_sample_step_m").value)
        self.path_clearance_skip_start_m = float(self.get_parameter("path_clearance_skip_start_m").value)
        self.goal_cost_check = bool(self.get_parameter("goal_cost_check").value)
        self.goal_max_cost = int(self.get_parameter("goal_max_cost").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[OccupancyGrid] = None
        self._warned_missing_global_costmap = False
        self.recent_goals: deque[Tuple[float, float]] = deque(maxlen=self.recent_goal_memory)
        self.goal_failures: dict[Tuple[int, int], int] = {}
        # World-frame XY of goals that have hit the failure threshold. Used for
        # radius-based blacklist veto in _compute_frontier_candidates.
        self.blacklisted_xy: List[Tuple[float, float]] = []
        self.active_goal: Optional[FrontierCandidate] = None
        self.active_goal_start = None
        self.no_path_cycles = 0
        self.last_costmap_clear_time = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator: Optional[BasicNavigator] = None
        if not self.dry_run:
            self.navigator = BasicNavigator()
            self.get_logger().info("Waiting for Nav2 to become active...")
            self.navigator.waitUntilNav2Active(localizer="bt_navigator")
            self.get_logger().info("Nav2 is active. Live goal mode enabled.")
        else:
            self.get_logger().info("Dry-run mode enabled. No goals will be sent.")

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_global_costmap, 10
        )
        self._tick_period = 1.0 / max(loop_hz, 0.1)

    def run(self) -> None:
        """Manual main loop. Avoids re-entering the executor from timer callbacks
        when BasicNavigator's blocking calls (getPath/isTaskComplete) need to spin."""
        last_tick = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = self.get_clock().now()
            if (now - last_tick).nanoseconds / 1e9 >= self._tick_period:
                try:
                    self._tick()
                except Exception as exc:
                    self.get_logger().warn(f"Tick failed: {exc}")
                last_tick = now

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self.global_costmap = msg

    def _tick(self) -> None:
        if self.map_msg is None:
            self.get_logger().info("Waiting for /map...")
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("No map->base_link TF yet; skipping this cycle.")
            return

        # Non-blocking live-goal monitoring: avoid nested spin loops in timer callback.
        if not self.dry_run and self.active_goal is not None:
            self._monitor_active_goal()
            return

        candidates, diag = self._compute_frontier_candidates(self.map_msg, robot_xy)
        if not candidates:
            self.get_logger().info(
                "No frontier candidates found. "
                f"free={diag['free']} unknown={diag['unknown']} occupied={diag['occupied']} "
                f"frontier_cells={diag['frontier_cells']} clusters={diag['clusters']} "
                f"rej_small={diag['rej_small']} rej_blacklist={diag['rej_blacklist']} "
                f"rej_recent={diag['rej_recent']} rej_min_dist={diag['rej_min_dist']} "
                f"rej_clearance={diag['rej_clearance']} rej_pocket={diag['rej_pocket']} "
                f"rej_goal_cost={diag['rej_goal_cost']}"
            )
            return

        if self.dry_run:
            best = candidates[0]
            self.get_logger().info(
                f"DRYRUN goal x={best.x:.2f}, y={best.y:.2f}, "
                f"cluster={best.cluster_size}, dist={best.distance_to_robot:.2f}, score={best.score:.2f}"
            )
            self.recent_goals.append((best.x, best.y))
            return

        assert self.navigator is not None
        best: Optional[FrontierCandidate] = None
        rejected_detour = 0
        rejected_no_path = 0
        rejected_blocked = 0
        for candidate in candidates:
            verdict = self._evaluate_path(robot_xy, candidate.x, candidate.y)
            if verdict == "ok":
                best = candidate
                break
            elif verdict == "detour":
                rejected_detour += 1
            elif verdict == "blocked":
                rejected_blocked += 1
            else:
                rejected_no_path += 1

        if best is None:
            self.get_logger().warn(
                f"No path-feasible frontier candidate found. "
                f"rejected_no_path={rejected_no_path} rejected_detour={rejected_detour} "
                f"rejected_blocked={rejected_blocked}"
            )
            self.no_path_cycles += 1
            self._maybe_recover_from_no_path()
            return

        # Approach heading: face the bearing from the robot's current pose to the
        # goal. Avoids a final spin-to-yaw=0 at the goal and naturally orients the
        # lidar toward the next slice of unknown space. Exploration only — for
        # post-mapping AMCL tasks (e.g. "face the beer can"), callers set their own
        # yaw via Nav2's normal interfaces.
        approach_yaw = math.atan2(best.y - robot_xy[1], best.x - robot_xy[0])
        goal = self._make_goal_pose(best.x, best.y, approach_yaw)
        self.get_logger().info(
            f"Sending goal x={best.x:.2f}, y={best.y:.2f}, yaw={approach_yaw:.2f}"
        )
        self.navigator.goToPose(goal)
        self.active_goal = best
        self.active_goal_start = self.get_clock().now()
        self.no_path_cycles = 0

    def _maybe_recover_from_no_path(self) -> None:
        if self.navigator is None:
            return
        if self.no_path_cycles < self.no_path_recovery_cycles:
            return

        now = self.get_clock().now()
        if self.last_costmap_clear_time is not None:
            elapsed = (now - self.last_costmap_clear_time).nanoseconds / 1e9
            if elapsed < self.costmap_clear_cooldown_sec:
                return

        self.get_logger().warn(
            f"No-path persisted for {self.no_path_cycles} cycles; clearing costmaps for recovery."
        )
        try:
            self.navigator.clearAllCostmaps()
            # Let previously failed regions be reconsidered after environment refresh.
            self.goal_failures.clear()
            self.recent_goals.clear()
            self.last_costmap_clear_time = now
            self.no_path_cycles = 0
        except Exception as exc:
            self.get_logger().warn(f"Costmap recovery failed: {exc}")

    def _monitor_active_goal(self) -> None:
        assert self.navigator is not None
        assert self.active_goal is not None

        try:
            if self.navigator.isTaskComplete():
                self._finalize_active_goal(self.navigator.getResult())
                return
        except (KeyboardInterrupt, TypeError):
            # Handle Ctrl+C races in action feedback conversion gracefully.
            self.get_logger().warn("Interrupted while checking task status; canceling active goal.")
            try:
                self.navigator.cancelTask()
            except Exception:
                pass
            self._finalize_active_goal(TaskResult.CANCELED)
            return

        if self.active_goal_start is None:
            self.active_goal_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.active_goal_start).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self.get_logger().warn("Goal timed out; canceling.")
            self.navigator.cancelTask()
            self._finalize_active_goal(TaskResult.CANCELED)

    def _finalize_active_goal(self, result: TaskResult) -> None:
        assert self.active_goal is not None
        key = self._goal_key(self.active_goal.x, self.active_goal.y)

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
            self.recent_goals.append((self.active_goal.x, self.active_goal.y))
            self.goal_failures.pop(key, None)
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal canceled.")
            self.goal_failures[key] = self.goal_failures.get(key, 0) + 1
        elif result == TaskResult.FAILED:
            self.get_logger().warn("Goal failed.")
            self.goal_failures[key] = self.goal_failures.get(key, 0) + 1
        else:
            self.get_logger().warn("Goal returned unknown status.")

        if key in self.goal_failures:
            self.get_logger().info(
                f"Goal failure count x={self.active_goal.x:.2f}, y={self.active_goal.y:.2f}: {self.goal_failures[key]}"
            )
            if self.goal_failures[key] >= self.failures_before_blacklist:
                xy = (self.active_goal.x, self.active_goal.y)
                if xy not in self.blacklisted_xy:
                    self.blacklisted_xy.append(xy)
                    self.get_logger().warn(
                        f"Blacklisting region around x={xy[0]:.2f}, y={xy[1]:.2f} "
                        f"(radius {self.blacklist_radius_m:.2f} m)."
                    )

        self.active_goal = None
        self.active_goal_start = None

    def _compute_frontier_candidates(
        self, msg: OccupancyGrid, robot_xy: Tuple[float, float]
    ) -> Tuple[List[FrontierCandidate], dict]:
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        free_cells: Set[Cell] = set()
        unknown_cells: Set[Cell] = set()
        occupied_cells: Set[Cell] = set()

        for y in range(height):
            row = y * width
            for x in range(width):
                v = data[row + x]
                if v == 0:
                    free_cells.add((x, y))
                elif v == -1:
                    unknown_cells.add((x, y))
                elif v >= 50:
                    occupied_cells.add((x, y))

        diag = {
            'free': len(free_cells),
            'unknown': len(unknown_cells),
            'occupied': len(occupied_cells),
            'frontier_cells': 0,
            'clusters': 0,
            'rej_small': 0,
            'rej_blacklist': 0,
            'rej_recent': 0,
            'rej_min_dist': 0,
            'rej_clearance': 0,
            'rej_pocket': 0,
            'rej_goal_cost': 0,
        }

        # 2026-05-17: Pre-compute search radius (in cells) for tight-passage clearance check.
        clearance_search_cells = max(1, int(math.ceil(self.min_clearance_m / res)) + 1)

        # Frontier = free cell adjacent to unknown space.
        frontier_cells: Set[Cell] = set()
        for ux, uy in unknown_cells:
            for nx, ny in ((ux - 1, uy), (ux + 1, uy), (ux, uy - 1), (ux, uy + 1)):
                if (nx, ny) in free_cells:
                    frontier_cells.add((nx, ny))

        diag['frontier_cells'] = len(frontier_cells)
        if not frontier_cells:
            return [], diag

        # Group frontier boundary cells into contiguous candidate regions.
        clusters = self._cluster_cells(frontier_cells)
        diag['clusters'] = len(clusters)
        candidates: List[FrontierCandidate] = []

        for cluster in clusters:
            if len(cluster) < self.min_cluster_size:
                diag['rej_small'] += 1
                continue

            # Use cluster centroid as a representative goal for that frontier region.
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res

            # Skip regions that have repeatedly failed to avoid ping-pong loops.
            key = self._goal_key(wx, wy)
            if self.goal_failures.get(key, 0) >= self.failures_before_blacklist:
                diag['rej_blacklist'] += 1
                continue
            # Radius-based blacklist: reject if within blacklist_radius_m of any
            # previously blacklisted goal. Catches near-variants of the same bad spot.
            if self.blacklist_radius_m > 0.0 and any(
                math.hypot(wx - bx, wy - by) < self.blacklist_radius_m
                for bx, by in self.blacklisted_xy
            ):
                diag['rej_blacklist'] += 1
                continue

            # Avoid immediately revisiting just-completed nearby goals.
            if self._too_close_to_recent(wx, wy):
                diag['rej_recent'] += 1
                continue

            dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if dist < self.min_goal_distance:
                diag['rej_min_dist'] += 1
                continue

            # 2026-05-17: Tight-passage avoidance. Measure distance from centroid to nearest
            # occupied cell within a small window. Reject if below min_clearance_m so the
            # explorer never picks goals tucked in alleys the robot can't comfortably escape.
            clearance = self._min_clearance_to_occupied(
                int(round(cx)), int(round(cy)), occupied_cells, clearance_search_cells, res
            )
            if clearance < self.min_clearance_m:
                diag['rej_clearance'] += 1
                continue

            # 2026-05-30: Goal-cell cost pre-check against the global costmap. The
            # planner refuses any goal whose cell is lethal/inflated (cost >=
            # inscribed). Filter here so we don't burn 24 getPath aborts per cycle
            # on candidates the planner can't accept.
            if self.goal_cost_check and self.global_costmap is not None:
                gc_cost = self._cost_at_world(self.global_costmap, wx, wy)
                if gc_cost is not None and gc_cost >= self.goal_max_cost:
                    diag['rej_goal_cost'] += 1
                    continue

            # 2026-05-17 (PM): "Get your nose out" — reject clusters whose unknown pocket
            # is already smaller than min_unknown_pocket_cells. Lidar has already seen the
            # boundaries; nothing new to map by squeezing in.
            if self.min_unknown_pocket_cells > 0:
                pocket = self._unknown_pocket_size(
                    cluster, unknown_cells, cap=self.min_unknown_pocket_cells * 2
                )
                if pocket < self.min_unknown_pocket_cells:
                    diag['rej_pocket'] += 1
                    continue

            # Prefer large frontier regions, penalize long detours, reward open clearance.
            # 2026-05-17: cluster bonus capped; distance penalty heavier — nearest-first bias.
            score = (
                min(float(len(cluster)), self.cluster_score_cap)
                - self.distance_score_weight * dist
                + self.clearance_score_weight * clearance
            )
            candidates.append(
                FrontierCandidate(
                    x=wx,
                    y=wy,
                    cluster_size=len(cluster),
                    distance_to_robot=dist,
                    score=score,
                )
            )

        # Highest score first; live mode still verifies planner reachability before send.
        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates, diag

    def _cost_at_world(
        self, cm: OccupancyGrid, wx: float, wy: float
    ) -> Optional[int]:
        """Return the global-costmap cell value (0-100) at world point (wx, wy),
        or None if outside the costmap. Negative values (-1 = unknown) returned
        as-is so callers can distinguish 'unknown' from 'lethal'."""
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        ix = int((wx - ox) / res)
        iy = int((wy - oy) / res)
        if ix < 0 or iy < 0 or ix >= cm.info.width or iy >= cm.info.height:
            return None
        return cm.data[iy * cm.info.width + ix]

    def _min_clearance_to_occupied(
        self,
        cx: int,
        cy: int,
        occupied: Set[Cell],
        search_cells: int,
        res: float,
    ) -> float:
        """Return distance (m) from cell (cx,cy) to nearest occupied cell within
        a square window of half-side search_cells. If no occupied cell is found
        in the window, returns a large value (treated as 'wide open')."""
        best_sq = None
        for dy in range(-search_cells, search_cells + 1):
            for dx in range(-search_cells, search_cells + 1):
                if (cx + dx, cy + dy) in occupied:
                    d_sq = dx * dx + dy * dy
                    if best_sq is None or d_sq < best_sq:
                        best_sq = d_sq
        if best_sq is None:
            return float(search_cells) * res  # nothing nearby; report window edge
        return math.sqrt(best_sq) * res

    def _unknown_pocket_size(
        self, cluster: List[Cell], unknown: Set[Cell], cap: int
    ) -> int:
        """Flood-fill connected unknown cells adjacent to the frontier cluster.

        Returns the count of connected unknown cells, capped at `cap` for speed.
        A small count means the lidar has already characterized the boundary of a
        tiny pocket; a large count means there's real unexplored area behind the
        frontier worth visiting.
        """
        seeds: List[Cell] = []
        for x, y in cluster:
            for n in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if n in unknown:
                    seeds.append(n)
        if not seeds:
            return 0
        visited: Set[Cell] = set()
        queue = deque(seeds)
        count = 0
        while queue and count < cap:
            cell = queue.popleft()
            if cell in visited or cell not in unknown:
                continue
            visited.add(cell)
            count += 1
            x, y = cell
            for n in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if n in unknown and n not in visited:
                    queue.append(n)
        return count

    def _cluster_cells(self, cells: Set[Cell]) -> List[List[Cell]]:
        remaining = set(cells)
        clusters: List[List[Cell]] = []

        while remaining:
            seed = next(iter(remaining))
            queue = deque([seed])
            remaining.remove(seed)
            cluster: List[Cell] = [seed]

            while queue:
                x, y = queue.popleft()
                for n in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                    if n in remaining:
                        remaining.remove(n)
                        queue.append(n)
                        cluster.append(n)

            clusters.append(cluster)

        return clusters

    def _too_close_to_recent(self, x: float, y: float) -> bool:
        for gx, gy in self.recent_goals:
            if math.hypot(x - gx, y - gy) < 0.35:
                return True
        return False

    def _goal_key(self, x: float, y: float) -> Tuple[int, int]:
        q = max(self.goal_key_resolution, 0.05)
        return (int(round(x / q)), int(round(y / q)))

    def _evaluate_path(self, robot_xy: Tuple[float, float], gx: float, gy: float) -> str:
        """Return 'ok', 'detour', 'blocked', or 'no_path'.

        'detour' means the global planner found a path but it is longer than
        max_path_detour_ratio * straight-line distance — i.e. routing the long way
        around (through chair-leg gauntlets, around the whole house, etc.).
        'blocked' means the returned path squeezes through a corridor of the global
        costmap whose cells are at/above path_clearance_max_cost (inflated obstacles).
        """
        assert self.navigator is not None
        start = self._make_goal_pose(robot_xy[0], robot_xy[1])
        goal = self._make_goal_pose(gx, gy)
        path = self.navigator.getPath(start, goal, use_start=True)
        if path is None or len(path.poses) < 2:
            return "no_path"
        straight = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        path_len = 0.0
        prev = path.poses[0].pose.position
        for ps in path.poses[1:]:
            p = ps.pose.position
            path_len += math.hypot(p.x - prev.x, p.y - prev.y)
            prev = p
        if self.max_path_detour_ratio > 0.0 and straight >= 0.2:
            ratio = path_len / straight
            if ratio > self.max_path_detour_ratio:
                self.get_logger().info(
                    f"Rejecting candidate x={gx:.2f}, y={gy:.2f}: "
                    f"path {path_len:.2f}m vs straight {straight:.2f}m (ratio {ratio:.2f} > {self.max_path_detour_ratio:.2f})"
                )
                return "detour"
        if self.path_clearance_check:
            verdict = self._check_path_clearance(path, gx, gy)
            if verdict == "blocked":
                return "blocked"
        return "ok"

    def _check_path_clearance(self, path, gx: float, gy: float) -> str:
        """Sample the global costmap along the path. Return 'blocked' if any sampled
        cell past path_clearance_skip_start_m has cost >= path_clearance_max_cost."""
        cm = self.global_costmap
        if cm is None:
            if not self._warned_missing_global_costmap:
                self.get_logger().warn(
                    "path_clearance_check enabled but /global_costmap/costmap not yet "
                    "received; skipping clearance check until it arrives."
                )
                self._warned_missing_global_costmap = True
            return "ok"
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        w = cm.info.width
        h = cm.info.height
        data = cm.data
        step = max(self.path_clearance_sample_step_m, res)
        skip = max(self.path_clearance_skip_start_m, 0.0)
        arc = 0.0
        last_sample_arc = -1.0
        max_cost = -1
        worst_xy = (0.0, 0.0)
        prev = path.poses[0].pose.position
        for ps in path.poses[1:]:
            p = ps.pose.position
            seg = math.hypot(p.x - prev.x, p.y - prev.y)
            arc += seg
            prev = p
            if arc < skip:
                continue
            if last_sample_arc >= 0.0 and (arc - last_sample_arc) < step:
                continue
            last_sample_arc = arc
            ix = int((p.x - ox) / res)
            iy = int((p.y - oy) / res)
            if ix < 0 or iy < 0 or ix >= w or iy >= h:
                continue
            cost = data[iy * w + ix]
            if cost > max_cost:
                max_cost = cost
                worst_xy = (p.x, p.y)
        if max_cost >= self.path_clearance_max_cost:
            self.get_logger().info(
                f"Rejecting candidate x={gx:.2f}, y={gy:.2f}: path squeezes through "
                f"cost={max_cost} at ({worst_xy[0]:.2f},{worst_xy[1]:.2f}) >= "
                f"{self.path_clearance_max_cost}"
            )
            return "blocked"
        return "ok"

    def _make_goal_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        assert self.navigator is not None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        # Quaternion for yaw rotation about Z axis.
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return None


def main() -> None:
    rclpy.init()
    node = FrontierExplorerV2()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # 2026-06-27: Cancel any in-flight Nav2 goal on shutdown so the BT
        # doesn't keep running its recovery sequence (ClearCostmaps -> Wait ->
        # BackUp x6) on a goal nobody is monitoring anymore. Without this, Ctrl-C
        # leaves Stormy struggling for minutes until a `dock` command preempts.
        try:
            if node.navigator is not None and node.active_goal is not None:
                node.get_logger().info("Shutdown: canceling active Nav2 goal.")
                node.navigator.cancelTask()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
