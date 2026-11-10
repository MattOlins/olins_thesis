"""ROS-free state machine and asynchronous global/local coordination."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .astar_planner import PlanningWorker
from .controller import LocalResult, VFHController
from .geometry import transform_point, wrap_angle
from .path_follower import PathFollower
from .progress_monitor import ProgressMonitor


@dataclass(frozen=True)
class NavigationResult:
    """One control cycle plus externally visible navigation state."""

    local: LocalResult
    state: str
    reason: str
    target: Optional[Tuple[float, float]] = None
    remaining: float = math.inf
    progress: float = 0.0
    turn_reversals: int = 0


class Navigator:
    """Share one local controller while owning episode and planning history."""

    def __init__(self, config, mode='vfh', worker=None):
        if mode not in ('vfh', 'vfh_astar'):
            raise ValueError('navigation_mode must be vfh or vfh_astar')
        self.config, self.mode = config, mode
        self.controller = VFHController(config)
        self.progress = ProgressMonitor(config)
        self.follower = PathFollower(config.lookahead_distance)
        self.worker = worker or PlanningWorker()
        self.grid = None
        self.map_revision = self.goal_revision = self.path_revision = 0
        self.reset()

    def reset(self, preserve_obstacles=False):
        """Invalidate old work and clear steering, route, and progress history."""
        self.goal_revision += 1
        self.path_revision += 1
        self.controller.reset(clear_obstacles=not preserve_obstacles)
        self.progress.reset()
        self.follower.reset()
        self.state, self.reason = 'WAITING', 'waiting_for_inputs'
        self.need_plan = True
        self.request_reason = 'new_goal'
        self.last_plan_time = -math.inf
        self.replan_count = self.stuck_replans = 0
        self.previous_signature = self.request_signature = None
        self.active_request_reason = None
        self.map_dirty = self.grid is not None
        self.last_time = None

    def set_map(self, grid):
        """Accept changed traversability snapshots only."""
        if self.grid is not None and self.grid.fingerprint == grid.fingerprint:
            return
        self.grid = grid
        self.map_revision += 1
        self.map_dirty = True

    def invalidate(self, reason):
        """Stop using the route immediately and request a replacement."""
        self.follower.reset()
        self.path_revision += 1
        if self.state in ('FAILED', 'SUCCEEDED'):
            return
        self.need_plan = True
        self.request_reason = reason
        self.state = 'REPLANNING' if self.replan_count else 'PLANNING'
        self.reason = reason
        self.controller.velocity.reset()
        self.progress.restart_window()

    def waiting(self, reason):
        """Suspend progress measurements while required inputs are unavailable."""
        self.progress.restart_window()
        if self.state not in ('FAILED', 'SUCCEEDED'):
            self.state, self.reason = 'WAITING', reason
        return NavigationResult(self.controller.stop(reason), self.state, self.reason,
                                turn_reversals=self.progress.turn_reversals)

    def _signature(self, path):
        return tuple(dict.fromkeys(self.grid.world_to_grid(p) for p in path))

    def _poll(self):
        completed = self.worker.poll()
        if completed is None:
            return
        token, result = completed
        if token != (self.goal_revision, self.map_revision):
            if not self.follower.path:
                self.need_plan = True
            return
        if self.state in ('FAILED', 'SUCCEEDED'):
            return
        if result.status == 'success':
            signature = self._signature(result.path)
            if self.active_request_reason == 'stuck' and signature == self.request_signature:
                self.stuck_replans += 1
            self.previous_signature = signature
            if self.stuck_replans >= self.config.max_stuck_replans:
                self.state, self.reason = 'FAILED', 'stuck_replan_limit'
                return
            self.follower.set_path(result.path)
            self.path_revision += 1
            self.progress.restart_window()
            self.state, self.reason = 'TRACKING', 'route_ready'
        else:
            self.state, self.reason = 'BLOCKED', result.status
            if self.active_request_reason == 'stuck':
                self.stuck_replans += 1
                if self.stuck_replans >= self.config.max_stuck_replans:
                    self.state, self.reason = 'FAILED', 'stuck_replan_limit'
                else:
                    self.need_plan = True

    def step(self, scan, pose, yaw_rate, final_goal, dt, now,
             map_pose=None, map_goal=None, map_to_odom=None):
        """Advance control; map inputs are required only for A* mode."""
        numbers = [pose.x, pose.y, pose.yaw, yaw_rate, *final_goal, dt, now]
        for transform in (map_pose, map_to_odom):
            if transform is not None:
                numbers.extend((transform.x, transform.y, transform.yaw))
        if map_goal is not None:
            numbers.extend(map_goal)
        if not all(math.isfinite(value) for value in numbers) or dt <= 0:
            return self.waiting('invalid_navigation_input')
        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now
        self._poll()
        if self.state in ('FAILED', 'SUCCEEDED'):
            return NavigationResult(self.controller.stop(self.reason), self.state, self.reason,
                                    turn_reversals=self.progress.turn_reversals)
        distance = math.dist((pose.x, pose.y), final_goal)
        if distance < self.config.goal_tolerance:
            self.state, self.reason = 'SUCCEEDED', 'goal_reached'
            return NavigationResult(self.controller.stop(self.reason), self.state,
                                    self.reason, remaining=distance,
                                    turn_reversals=self.progress.turn_reversals)
        target, remaining = final_goal, distance
        if self.mode == 'vfh_astar':
            if self.grid is None:
                return self.waiting('missing_map')
            if map_pose is None or map_goal is None or map_to_odom is None:
                return self.waiting('missing_map_transform')
            position = map_pose.x, map_pose.y
            if self.map_dirty:
                if self.follower.path:
                    if not self.grid.path_clear((position, *self.follower.remaining_path())):
                        self.invalidate('map_invalidated_route')
                elif self.worker.future is None:
                    self.need_plan = True
                self.map_dirty = False
            if self.follower.path:
                follow = self.follower.update(position, self.grid)
                if follow.target is None:
                    self.invalidate('no_visible_waypoint')
                elif follow.deviation > self.config.route_deviation:
                    self.invalidate('route_deviation')
                else:
                    target = transform_point(follow.target, map_to_odom)
                    remaining = follow.remaining
            if not self.follower.path:
                if self.need_plan and now-self.last_plan_time >= self.config.replan_interval:
                    token = self.goal_revision, self.map_revision
                    if self.worker.submit(self.grid, position, map_goal, token):
                        self.last_plan_time = now
                        self.need_plan = False
                        self.request_signature = self.previous_signature
                        self.active_request_reason = self.request_reason
                        self.replan_count += 1
                        self.state = 'PLANNING' if self.replan_count == 1 else 'REPLANNING'
                        self.reason = self.request_reason
                self.progress.restart_window()
                return NavigationResult(self.controller.stop(self.reason), self.state, self.reason,
                                        turn_reversals=self.progress.turn_reversals)
        local = self.controller.step(scan, pose, yaw_rate, target, dt)
        bearing = wrap_angle(math.atan2(target[1]-pose.y, target[0]-pose.x)-pose.yaw)
        result = self.progress.update(
            now, remaining, local.command.angular, alignment_error=bearing, yaw=pose.yaw)
        if result.progress >= self.config.minimum_progress:
            self.stuck_replans = 0
        if result.stuck:
            if self.mode == 'vfh_astar':
                self.invalidate('stuck')
            else:
                self.state, self.reason = 'FAILED', 'stuck_without_global_planner'
            local = self.controller.stop(self.reason)
        else:
            self.state = 'TRACKING' if local.reason == 'ok' else 'BLOCKED'
            self.reason = local.reason
        return NavigationResult(local, self.state, self.reason, target, remaining,
                                result.progress, result.turn_reversals)

    def close(self):
        """Join the planning worker before node destruction."""
        self.worker.close()
