"""Explicit seeded RRT: sampling, nearest neighbour, steering and parents."""
import math
import random


class PlanningError(RuntimeError):
    pass


class RRTPlanner:
    def __init__(self, grid, seed=11, step_size=0.6, goal_bias=0.15,
                 goal_tolerance=0.8, max_iterations=10000):
        if (not all(math.isfinite(v) for v in (step_size, goal_bias, goal_tolerance))
                or step_size <= 0 or not 0 <= goal_bias <= 1
                or goal_tolerance <= 0 or max_iterations <= 0):
            raise ValueError('Invalid RRT parameters')
        self.grid = grid
        self.rng = random.Random(seed)
        self.step_size, self.goal_bias = step_size, goal_bias
        self.goal_tolerance, self.max_iterations = goal_tolerance, max_iterations
        self.nodes, self.parents, self.iterations = [], [], 0

    def reconstruct(self, index):
        path = []
        while index != -1:
            path.append(self.nodes[index])
            index = self.parents[index]
        return path[::-1]

    def plan(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        for name, point in [('A', start), ('B', goal)]:
            if not self.grid.point_free(point):
                raise ValueError(f'{name} outside map or inside inflated obstacle')
        if math.dist(start, goal) < 1e-9:
            raise ValueError('A and B must be different')
        self.nodes, self.parents = [start], [-1]
        x0, x1, y0, y1 = self.grid.bounds
        for self.iterations in range(1, self.max_iterations+1):
            sample = goal if self.rng.random() < self.goal_bias else (
                self.rng.uniform(x0, x1), self.rng.uniform(y0, y1))
            if not self.grid.point_free(sample):
                continue
            nearest = min(range(len(self.nodes)), key=lambda i: math.dist(self.nodes[i], sample))
            p = self.nodes[nearest]
            distance = math.dist(p, sample)
            if distance < 1e-9:
                continue
            ratio = min(1.0, self.step_size/distance)
            q = tuple(p[k]+ratio*(sample[k]-p[k]) for k in (0, 1))
            if not self.grid.segment_free(p, q):
                continue
            self.nodes.append(q)
            self.parents.append(nearest)
            if math.dist(q, goal) <= self.goal_tolerance and self.grid.segment_free(q, goal):
                index = len(self.nodes)-1
                if math.dist(q, goal) > 1e-9:
                    self.nodes.append(goal)
                    self.parents.append(index)
                path = self.reconstruct(len(self.nodes)-1)
                if not self.grid.validate_path(path):
                    raise PlanningError('Internal path validation failed')
                return path
        raise PlanningError(f'No route after {self.max_iterations} iterations')
