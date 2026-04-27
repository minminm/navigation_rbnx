---
name: navigation
description: Navigate the GO2 robot to a target position using Nav2.
---

# Navigation Skill

Send the robot to any (x, y) coordinate on the map with a specified heading.

## Available Tools

- `base_navigate(pose)` — send a navigation goal, returns a `goal_id`
- `base_nav_status(goal_id)` — check goal progress (QUEUED / ACCEPTED / SUCCEEDED / ABORTED)
- `base_nav_cancel(goal_id)` — cancel an in-progress navigation

## Typical Workflow

1. Call `base_navigate` with target pose — returns `goal_id`
2. Poll `base_nav_status(goal_id)` until status is `SUCCEEDED` or `ABORTED`
3. If stuck or timeout, call `base_nav_cancel(goal_id)` and try alternative

## Coordinate System

- Frame: `map` (default)
- Yaw: radians, 0 = facing +x, counter-clockwise positive

## Dependencies

- mapping_rbnx must be running (provides odom, occupancy_grid, scan_2d)
- Nav2 stack must be healthy (planner + controller + BT navigator)

## Notes

- Navigation requires the robot to be localized first (mapping_rbnx in localization mode)
- The robot may fail near tight obstacles; increase clearance or cancel and retry
