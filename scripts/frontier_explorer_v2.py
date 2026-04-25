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
        self.declare_parameter("min_cluster_size", 6)
        self.declare_parameter("min_goal_distance_m", 0.6)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("recent_goal_memory", 6)
        self.declare_parameter("failures_before_blacklist", 2)
        self.declare_parameter("goal_key_resolution_m", 0.2)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        loop_hz = float(self.get_parameter("loop_hz").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance_m").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.recent_goal_memory = int(self.get_parameter("recent_goal_memory").value)
        self.failures_before_blacklist = int(self.get_parameter("failures_before_blacklist").value)
        self.goal_key_resolution = float(self.get_parameter("goal_key_resolution_m").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.recent_goals: deque[Tuple[float, float]] = deque(maxlen=self.recent_goal_memory)
        self.goal_failures: dict[Tuple[int, int], int] = {}
        self.active_goal: Optional[FrontierCandidate] = None
        self.active_goal_start = None

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
        period = 1.0 / max(loop_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

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

        candidates = self._compute_frontier_candidates(self.map_msg, robot_xy)
        if not candidates:
            self.get_logger().info("No frontier candidates found.")
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
        for candidate in candidates:
            if self._path_exists(robot_xy, candidate.x, candidate.y):
                best = candidate
                break

        if best is None:
            self.get_logger().warn("No path-feasible frontier candidate found.")
            return

        goal = self._make_goal_pose(best.x, best.y)
        self.get_logger().info(f"Sending goal x={best.x:.2f}, y={best.y:.2f}")
        self.navigator.goToPose(goal)
        self.active_goal = best
        self.active_goal_start = self.get_clock().now()

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

        self.active_goal = None
        self.active_goal_start = None

    def _compute_frontier_candidates(
        self, msg: OccupancyGrid, robot_xy: Tuple[float, float]
    ) -> List[FrontierCandidate]:
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        free_cells: Set[Cell] = set()
        unknown_cells: Set[Cell] = set()

        for y in range(height):
            row = y * width
            for x in range(width):
                v = data[row + x]
                if v == 0:
                    free_cells.add((x, y))
                elif v == -1:
                    unknown_cells.add((x, y))

        frontier_cells: Set[Cell] = set()
        for ux, uy in unknown_cells:
            for nx, ny in ((ux - 1, uy), (ux + 1, uy), (ux, uy - 1), (ux, uy + 1)):
                if (nx, ny) in free_cells:
                    frontier_cells.add((nx, ny))

        if not frontier_cells:
            return []

        clusters = self._cluster_cells(frontier_cells)
        candidates: List[FrontierCandidate] = []

        for cluster in clusters:
            if len(cluster) < self.min_cluster_size:
                continue

            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res

            key = self._goal_key(wx, wy)
            if self.goal_failures.get(key, 0) >= self.failures_before_blacklist:
                continue

            if self._too_close_to_recent(wx, wy):
                continue

            dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if dist < self.min_goal_distance:
                continue

            score = float(len(cluster)) - 0.7 * dist
            candidates.append(
                FrontierCandidate(
                    x=wx,
                    y=wy,
                    cluster_size=len(cluster),
                    distance_to_robot=dist,
                    score=score,
                )
            )

        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates

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

    def _path_exists(self, robot_xy: Tuple[float, float], gx: float, gy: float) -> bool:
        assert self.navigator is not None
        start = self._make_goal_pose(robot_xy[0], robot_xy[1])
        goal = self._make_goal_pose(gx, gy)
        path = self.navigator.getPath(start, goal, use_start=True)
        return path is not None and len(path.poses) > 1

    def _make_goal_pose(self, x: float, y: float) -> PoseStamped:
        assert self.navigator is not None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
