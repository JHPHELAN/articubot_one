#!/usr/bin/env bash
# record_explore.sh — one command to launch the frontier explorer AND
# record everything needed to tune it offline.
#
# Behaviour:
#   * Creates $HOME/robot_logs/explore/explore_<STAMP>/ (override with
#     RECORD_EXPLORE_ROOT env var).
#   * Starts a rosbag2 (mcap) recorder as a background child capturing
#     the topics the explorer's decisions depend on.
#   * Launches `ros2 launch frontier_explorer explore.launch.py "$@"` in
#     the foreground so you can WATCH the diag lines scroll. Any extra
#     args are passed straight through, e.g.
#         record_explore params_file:=/path/to/my.yaml dry_run:=true
#   * Tees the explorer's stdout+stderr to explorer.log in the run dir.
#   * ~10 s after launch, snapshots the live parameters via
#     `ros2 param dump /frontier_explorer`.
#   * On Ctrl-C, stops the bag cleanly (SIGINT + wait) before exit.
#
# No cameras are recorded — this is exploration telemetry only, and
# bags stay small (~tens to low hundreds of MB per 20-minute run).

set -u

STAMP=$(date +%Y%m%d_%H%M%S)
OUT_ROOT="${RECORD_EXPLORE_ROOT:-$HOME/robot_logs/explore}"
OUT_DIR="$OUT_ROOT/explore_$STAMP"
BAG_DIR="$OUT_DIR/bag"
NOTES_DIR="$OUT_DIR/notes"
EXPLORER_LOG="$OUT_DIR/explorer.log"
BAG_LOG="$OUT_DIR/bag.log"

mkdir -p "$NOTES_DIR"

# Sniff for params_file:=... in the passthrough args so we can copy the
# YAML into the run's notes. The value still gets forwarded to ros2 launch
# regardless.
PARAMS_FILE=""
for arg in "$@"; do
    case "$arg" in
        params_file:=*) PARAMS_FILE="${arg#params_file:=}" ;;
    esac
done

# Topics recorded. Includes /oak/points (raw depth pointcloud that feeds
# the local_costmap voxel_layer — necessary to diagnose sensing failures
# like the 2026-09-28 hall-table cross-plank wedge and living-room shoes
# trip) and /oak/rgb/image_rect/compressed (~2 Mbps, gives visual context
# for post-mortem review). Both /amcl_pose and /pose because different
# localizer configs publish one or the other; empty subs are harmless.
TOPICS=(
    /map
    /global_costmap/costmap
    /local_costmap/costmap
    /tf
    /tf_static
    /odom
    /amcl_pose
    /pose
    /plan
    /local_plan
    /scan
    /oak/points
    /oak/rgb/image_rect/compressed
    /rosout
    /behavior_tree_log
    /goal_pose
    /navigate_to_pose/_action/status
    /navigate_to_pose/_action/feedback
)

# ---- notes: the "what produced this bag" snapshot ----
{
    echo "# record_explore run notes"
    echo "stamp:  $STAMP"
    echo "host:   $(uname -n)"
    echo "kernel: $(uname -rms)"
    echo "user:   $(id -un)"
    echo "cwd:    $PWD"
    echo
    echo "## invocation"
    printf '%q ' "$0" "$@"; echo
    echo
    echo "## articubot_one git"
    if [[ -d "$HOME/robot_ws/src/articubot_one/.git" ]]; then
        (cd "$HOME/robot_ws/src/articubot_one" && git log -1 --oneline && git status --porcelain)
    else
        echo "(not a git repo)"
    fi
    echo
    echo "## frontier_explorer git"
    if [[ -d "$HOME/robot_ws/src/frontier_explorer/.git" ]]; then
        (cd "$HOME/robot_ws/src/frontier_explorer" && git log -1 --oneline && git status --porcelain)
    else
        echo "(not a git repo)"
    fi
    echo
    echo "## params_file (from CLI)"
    echo "${PARAMS_FILE:-(none - using launch default)}"
} > "$NOTES_DIR/run.txt"

if [[ -n "$PARAMS_FILE" && -f "$PARAMS_FILE" ]]; then
    cp -- "$PARAMS_FILE" "$NOTES_DIR/params_at_launch.yaml"
fi

# ---- cleanup / signal handling ----
BAG_PID=""
PARAM_DUMP_PID=""
cleanup() {
    if [[ -n "$PARAM_DUMP_PID" ]] && kill -0 "$PARAM_DUMP_PID" 2>/dev/null; then
        kill "$PARAM_DUMP_PID" 2>/dev/null || true
    fi
    if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        echo "[record_explore] stopping bag (pid $BAG_PID)..."
        kill -INT "$BAG_PID" 2>/dev/null || true
        # Give rosbag2 a few seconds to flush; force-terminate if it hangs.
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$BAG_PID" 2>/dev/null || break
            sleep 0.5
        done
        kill -TERM "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
    echo "[record_explore] run saved under: $OUT_DIR"
}
trap cleanup EXIT INT TERM

echo "[record_explore] output dir: $OUT_DIR"
echo "[record_explore] starting rosbag2 recorder (mcap)..."
ros2 bag record -s mcap -o "$BAG_DIR" "${TOPICS[@]}" \
    > "$BAG_LOG" 2>&1 &
BAG_PID=$!

# Let the recorder finish opening subscriptions before we start publishing
# the first goal. Two seconds is comfortably enough on a Pi 5.
sleep 2

# Snapshot live parameters ~10 s in, once the node has settled. Background
# so we don't hold up the foreground launch.
(
    sleep 10
    ros2 param dump /frontier_explorer > "$NOTES_DIR/params_live.yaml" 2>/dev/null || true
) &
PARAM_DUMP_PID=$!

echo "[record_explore] launching explorer... (Ctrl-C to stop everything)"
# Tee stdout+stderr into explorer.log. When user hits Ctrl-C, ros2 launch
# catches SIGINT and shuts down cleanly; the trap above then handles bag.
ros2 launch frontier_explorer explore.launch.py "$@" 2>&1 | tee "$EXPLORER_LOG"

echo "[record_explore] explorer exited; cleanup trap will finalize bag."
