# `frontier_explorer_v2.py` — Frontier-Based Autonomous Exploration for Nav2

This README describes the autonomous exploration node used on the `exploration`
branch:

- [`scripts/frontier_explorer_v2.py`](scripts/frontier_explorer_v2.py)

The explanation below was written by **ChatGPT** (via Sergei Grachine of the
HomeBrew Robotics Club) as an answer to Michael Wimble's question about how the
node works. It is reproduced here, lightly reformatted for GitHub Markdown, so
that HBRC members and anyone else browsing the branch has a single canonical
place to read it. Original shared conversation:
<https://chatgpt.com/s/t_6a979066b6288191b1f8a24cc591ae11>

The generic upstream package README that used to live here is preserved on the
`main` / `dev` branches of the fork (and in the upstream
[`slgrobotics/articubot_one`](https://github.com/slgrobotics/articubot_one)
repository). This branch replaces it with something branch-specific because the
exploration node is what makes this branch interesting.

---

## In one sentence

It repeatedly does:

> Look at the SLAM `/map` → find boundaries between mapped free space and
> unknown space → form candidate exploration regions → reject unsafe or
> unproductive ones → choose the best remaining frontier → ask Nav2 whether it
> can reach it sensibly → send it as a `NavigateToPose` goal → monitor the
> result → repeat.

That is the standard frontier-exploration concept, but this implementation has
quite a few practical heuristics to keep the robot from getting stuck exploring
silly places.

---

## 1. What it consumes

The important inputs are:

```
/map
/global_costmap/costmap
map → base_link TF
Nav2
```

`/map` is the SLAM occupancy grid. The node classifies every cell as:

```
 0       free
-1       unknown
>= 50    occupied
```

It also subscribes to Nav2's global costmap, which gives it information about
inflated obstacles and areas that Nav2 considers problematic.

It obtains the robot's current position with:

```
TF: map → base_link
```

and therefore operates entirely in the map coordinate system.

---

## 2. What is a "frontier"?

The key idea is:

```
                 UNKNOWN
   ? ? ? ? ? ? ? ? ? ? ? ? ?
   ? ? ? ? ? ? ? ? ? ? ? ? ?
   ? ? ? ? ? ? ? ? ? ? ? ? ?

             ↑ frontier

   . . . . . . . . . . . . .
   . . . . ROBOT . . . . . .
   . . . . . . . . . . . . .

                 FREE
```

A **frontier** is a free map cell immediately adjacent to an unknown cell.

The code explicitly constructs it that way:

```python
for ux, uy in unknown_cells:
    for nx, ny in neighbors:
        if (nx, ny) in free_cells:
            frontier_cells.add((nx, ny))
```

So it isn't trying to understand rooms, doors, objects, etc. It simply asks:

> Where does known free space touch unexplored space?

That's the fundamental exploration algorithm.

---

## 3. It groups frontier cells into regions

Suppose the map has something like:

```
########################
#                      #
#     ROBOT            #
#                      #
#########     ##########
        #     #
        # ??? #
        # ??? #
        #######
```

There may be dozens or hundreds of individual frontier cells. The program
groups adjacent frontier cells into clusters. So instead of considering:

```
frontier cell 1
frontier cell 2
frontier cell 3
...
frontier cell 200
```

it gets something like:

```
Frontier A: 42 cells
Frontier B: 17 cells
Frontier C: 63 cells
Frontier D:  5 cells
```

It then treats each cluster as a potential exploration destination.

---

## 4. It uses the cluster centroid as the goal

For each frontier cluster, it calculates the centroid:

```python
cx = sum(c[0] for c in cluster) / len(cluster)
cy = sum(c[1] for c in cluster) / len(cluster)
```

and converts that map-grid coordinate into world coordinates (`wx`, `wy`).

Conceptually:

```
          frontier cluster
       ###################
      #####################
      #####################
             ↑
          centroid
             |
             ↓
          GOAL POINT
```

This is an important design choice: it doesn't pick the unknown cell itself as
the goal. It picks a free-space cell representing the frontier.

---

## 5. It rejects tiny frontiers

A tiny frontier might just be noise or a tiny sliver of unknown space. For
example:

```
     ###
     ###
      #
      ?
```

There is little reason to send the robot there. So it has:

```python
min_cluster_size
```

and rejects frontier clusters smaller than that threshold.

---

## 6. It deliberately avoids tight spaces

This is one of the more interesting additions. The code calculates the distance
from the proposed goal to the nearest occupied map cell. The default is:

```python
min_clearance_m = 0.40
```

So a frontier whose centroid is too close to an obstacle is rejected. The
comments explicitly describe this as avoiding tight alleys and places where
the robot could squeeze between obstacles.

Conceptually:

```
          chair
       ##########
       ##########

            X       ← frontier candidate
          /
        too close

        ROBOT
         ↓
       [   ]
```

This is particularly useful for a robot of Stingray's size, because a pure
frontier algorithm will happily try to explore the unknown space underneath
furniture or between chair legs.

---

## 7. It also looks behind the frontier

This is another useful heuristic. Imagine:

```
################ chair
       ? ?
       ? ?      ← tiny unknown pocket
       ? ?
###########
     ^
     frontier
```

A lidar may leave tiny patches of unknown space around furniture.

The program flood-fills the unknown area behind a frontier and measures its
size. If the unknown "pocket" is too small:

```python
min_unknown_pocket_cells = 25
```

it rejects the frontier. The reasoning is essentially:

> "I've already characterized this little nook. Don't stick the robot's nose
> into it just because a few unknown cells remain."

That's a very practical improvement over textbook frontier exploration.

---

## 8. It rejects previously failed areas

Suppose Nav2 repeatedly fails to reach `(17.5, 3.3)`. A naive frontier explorer
might keep selecting:

```
(17.5, 3.3)
(17.6, 3.4)
(17.4, 3.2)
...
```

because the frontier is still there.

This implementation keeps a **failure count** for goals. After enough failures
it blacklists a whole region, rather than just one exact grid cell. The default
blacklist radius is:

```python
blacklist_radius_m = 0.8
```

So:

```
             X
       X     X     X
          XXXXX
        XXXXXXX
          XXXXX
             ↑
       known bad region
```

gets avoided.

---

## 9. It scores the remaining candidates

The basic score is:

```python
score = (
    cluster_size
    - distance_score_weight * distance
    + clearance_score_weight * clearance
)
```

with the cluster bonus capped.

In other words, it prefers:

- **Large frontier** — more unknown space to reveal
- **Close to robot** — cheaper to reach
- **Lots of obstacle clearance** — safer to approach

So this isn't simply "go to the nearest frontier." It's closer to:

> "Go to a reasonably large, nearby, open frontier."

---

## 10. But it doesn't blindly trust that score

This is probably the most important part of `v2`.

After ranking the candidates, it asks Nav2's global planner for an actual path:

```python
path = self.navigator.getPath(start, goal)
```

Then it examines that path.

This catches situations like:

```
Robot                       Frontier
  ↓                            ↓

  R . . . . . . . . .          X
      ###########
      ###########
      ###########

straight line looks short
BUT
actual path must go:

R → → → → → → around the entire building → X
```

The program calculates:

```
actual path length
------------------
straight-line distance
```

and rejects the goal if that ratio exceeds:

```python
max_path_detour_ratio = 3.0
```

So a frontier that is geometrically close but requires an absurd detour is
rejected.

---

## 11. It checks the actual path for narrow corridors

There's another Nav2-specific safety filter. After getting a path, it samples
the global costmap along the path. If it encounters a sufficiently high cost:

```python
path_clearance_max_cost
```

the candidate is rejected.

The purpose is to catch something like:

```
         chair
      ##########
           \
            \ path
             \
              X
```

where the global planner technically found a path, but that path squeezes
through an undesirable inflated-obstacle region. The code specifically
describes this as avoiding "corridor squeezes."

---

## 12. It also checks the goal itself against the costmap

This is a particularly useful fix for frontier exploration. The frontier
centroid might be free according to `/map` but still be `unknown` or
`lethal / inflated` in the Nav2 global costmap.

The code therefore checks `/global_costmap/costmap` before attempting the
goal. It rejects:

- unknown goal cells
- cells whose cost is above the configured maximum

This prevents a lot of pointless Nav2 goal failures.

---

## 13. Eventually it sends the goal to Nav2

Once all those filters have passed:

```python
goal = self._make_goal_pose(...)
self.navigator.goToPose(goal)
```

So this node does not drive the robot itself. It is a **high-level
exploration planner**. The division of labor is:

```
                frontier_explorer_v2
                         │
                         │ "Go explore here"
                         ▼
                 Nav2 NavigateToPose
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
       global planner          local controller
              │                     │
              └──────────┬──────────┘
                         ▼
                       robot
```

Nav2 handles the actual path planning, obstacle avoidance, controller
behavior, recovery behaviors, etc.

---

## 14. It deliberately chooses the approach heading

The goal orientation isn't arbitrary. It computes:

```python
approach_yaw = atan2(
    goal_y - robot_y,
    goal_x - robot_x,
)
```

So the robot arrives approximately facing toward the frontier. That's useful
for exploration because the lidar / camera is then pointed toward the newly
unexplored area. The comments explicitly say this avoids ending with an
unnecessary spin to a fixed yaw and naturally points the lidar toward the
next unknown region.

---

## 15. Then it waits for Nav2 to finish

While a goal is active, it doesn't keep generating new goals. It does:

```
goal active?
    │
    ├── yes → monitor Nav2
    │
    └── no  → find next frontier
```

The main loop runs at a configurable rate and processes `/map`, costmap, TF,
etc.

When Nav2 reports success, cancellation, or failure, the node updates its
state. Failures increment the goal failure counter, and sufficiently bad /
repeated goals become blacklisted.

---

## 16. It has a recovery mechanism

There's an additional escape hatch. If it goes through several cycles without
finding a path-feasible frontier, it eventually calls:

```python
self.navigator.clearAllCostmaps()
```

and clears some of its exploration memory:

```python
self.goal_failures.clear()
self.recent_goals.clear()
```

The idea is:

> "Maybe the costmap is stale or got into a bad state. Refresh everything
> and try again."

---

## 17. It has a dry-run mode

This is nice for development. In dry-run mode it:

```
/map
   ↓
find frontiers
   ↓
score / filter them
   ↓
print selected goal
   ↓
DO NOT MOVE ROBOT
```

The code explicitly logs something like:

```
DRYRUN goal x=..., y=..., cluster=..., dist=..., score=...
```

instead of calling `goToPose()`.

That makes it possible to tune the exploration algorithm against a map
without actually driving the robot.

---

## Overall architecture

```
                    SLAM
                     │
                     ▼
                   /map
                     │
                     ▼
          ┌──────────────────────┐
          │ Classify map cells   │
          │ free / unknown /     │
          │ occupied             │
          └──────────┬───────────┘
                     │
                     ▼
             Find frontier cells
                     │
                     ▼
              Cluster frontiers
                     │
                     ▼
             Calculate centroids
                     │
          ┌──────────┴───────────┐
          │      FILTERS         │
          │                      │
          │ • too small?         │
          │ • blacklisted?       │
          │ • too recent?        │
          │ • too close?         │
          │ • insufficient       │
          │   clearance?         │
          │ • tiny unknown       │
          │   pocket?            │
          │ • bad costmap cell?  │
          └──────────┬───────────┘
                     │
                     ▼
                Score goals
                     │
                     ▼
             Rank candidates
                     │
                     ▼
             Ask Nav2 planner
                     │
          ┌──────────┴──────────┐
          │                     │
       no path              path found
          │                     │
       reject                  ▼
                            Check path
                            detour ratio
                            + clearance
                                 │
                                 ▼
                          ┌─────────────┐
                          │ Nav2 goal   │
                          │ NavigateTo  │
                          │ Pose        │
                          └──────┬──────┘
                                 │
                                 ▼
                              Robot
                                 │
                                 ▼
                              /map
                                 │
                                 └─────── repeat
```

### The important conceptual point

This is **not** a SLAM algorithm. It assumes that something else — SLAM
Toolbox, RTAB-Map, etc. — is producing `/map`.

It is **not** a local obstacle avoidance algorithm either. Nav2 handles that.

Its job is essentially:

> "Given everything the robot has mapped so far, where should I send the
> robot next to learn something new?"

And `v2` has been heavily engineered around the practical problems that a
naive frontier explorer encounters: chair legs, tiny unknown pockets,
unreachable frontiers, absurd detours, stale costmaps, repeated failed
goals, and goal cells that look valid in `/map` but aren't valid to Nav2.

---

## Attribution

Explanation authored by **ChatGPT** (OpenAI), shared by Sergei Grachine of
the [HomeBrew Robotics Club](https://www.hbrobotics.org/) and published here
in response to a question from Michael Wimble at an HBRC Zoom meeting on
2026-09-01. The code being described lives at
[`scripts/frontier_explorer_v2.py`](scripts/frontier_explorer_v2.py) on this
branch and was written with the help of GitHub Copilot (Claude) working with
JHPHELAN.

Original shared ChatGPT conversation:
<https://chatgpt.com/s/t_6a979066b6288191b1f8a24cc591ae11>
