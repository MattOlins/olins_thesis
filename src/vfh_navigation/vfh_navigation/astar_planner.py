"""Deterministic eight-connected A* and a single immutable-snapshot worker."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import heapq
import itertools
import math


@dataclass(frozen=True)
class PlannerResult:
    """Explicit search result; successful paths contain map-frame coordinates."""

    status: str
    path: tuple = ()
    expanded: int = 0


def plan(grid, start, goal):
    """Find an optimal grid route without snapping invalid requested endpoints."""
    if grid is None:
        return PlannerResult('invalid_map')
    if not all(math.isfinite(v) for v in (*start, *goal)):
        return PlannerResult('invalid_coordinates')
    first, last = grid.world_to_grid(start), grid.world_to_grid(goal)
    for name, cell, point in (('start', first, start), ('goal', last, goal)):
        status = grid.cell_status(cell)
        if status != 'free':
            return PlannerResult(f'{name}_{status}')
        if not grid.segment_clear(point, point):
            return PlannerResult(f'{name}_insufficient_clearance')

    def heuristic(cell):
        dx, dy = abs(cell[0]-last[0]), abs(cell[1]-last[1])
        return max(dx, dy) + (math.sqrt(2)-1)*min(dx, dy)

    serial = itertools.count()
    queue = [(heuristic(first), next(serial), 0.0, first)]
    distance = {first: 0.0}
    parent = {}
    expanded = 0
    while queue:
        _, _, cost, cell = heapq.heappop(queue)
        if cost > distance[cell]+1e-12:
            continue
        expanded += 1
        if cell == last:
            route = [cell]
            while route[-1] != first:
                route.append(parent[route[-1]])
            centers = [grid.grid_to_world(p) for p in reversed(route)]
            # Preserve exact endpoints so the follower reaches the actual requested goal.
            points = [tuple(start), *centers, tuple(goal)]
            route = tuple(p for i, p in enumerate(points) if i == 0 or p != points[i-1])
            if not grid.path_clear(route):
                return PlannerResult('invalid_endpoint_connection', expanded=expanded)
            return PlannerResult('success', route, expanded)
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1),
                       (1, 1), (-1, 1), (-1, -1), (1, -1)):
            neighbor = cell[0]+dx, cell[1]+dy
            if not grid.is_free(neighbor):
                continue
            if dx and dy and (not grid.is_free((cell[0]+dx, cell[1])) or
                              not grid.is_free((cell[0], cell[1]+dy))):
                continue
            tentative = cost + (math.sqrt(2) if dx and dy else 1.0)
            if tentative+1e-12 < distance.get(neighbor, math.inf):
                if not grid.segment_clear(grid.grid_to_world(cell), grid.grid_to_world(neighbor)):
                    continue
                distance[neighbor] = tentative
                parent[neighbor] = cell
                heapq.heappush(queue, (
                    tentative+heuristic(neighbor), next(serial), tentative, neighbor))
    return PlannerResult('unreachable', expanded=expanded)


class PlanningWorker:
    """Keep at most one running search; only the coordinator consumes its result."""

    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='astar')
        self.future = None
        self.token = None

    def submit(self, grid, start, goal, token):
        """Submit a snapshot if the worker has no unfinished request."""
        if self.future is not None:
            return False
        self.token = token
        self.future = self.executor.submit(plan, grid, tuple(start), tuple(goal))
        return True

    def poll(self):
        """Return the request token and result once, without blocking the control loop."""
        if self.future is None or not self.future.done():
            return None
        try:
            result = self.future.result()
        except Exception:
            result = PlannerResult('planner_error')
        token = self.token
        self.future = None
        return token, result

    def close(self):
        """Wait for the single bounded grid search during shutdown."""
        self.executor.shutdown(wait=True, cancel_futures=True)
