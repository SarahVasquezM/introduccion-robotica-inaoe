import json
from pathlib import Path
import time
from .rrt_planner import RRTPlanner
from .path_smoothing import smooth_path
from .trajectory_utils import path_length, segment_times


def plan(grid, start, goal, total_time, seed, step_size=0.6, goal_bias=0.15,
         goal_tolerance=0.8, max_iterations=10000, smoothing_attempts=200):
    planner = RRTPlanner(grid, seed, step_size, goal_bias, goal_tolerance, max_iterations)
    begun = time.monotonic()
    raw = planner.plan(start, goal)
    smoothed = smooth_path(raw, grid, smoothing_attempts, seed)
    times = segment_times(smoothed, total_time)
    return dict(world_config=grid.config, start=list(start), goal=list(goal), seed=seed,
                raw=raw, smoothed=smoothed, nodes=planner.nodes, parents=planner.parents,
                iterations=planner.iterations, raw_length=path_length(raw),
                smoothed_length=path_length(smoothed), segment_times=times,
                total_time=total_time, real_time=None, execution_status='not_executed',
                planning_wall_seconds=time.monotonic()-begun,
                parameters=dict(step_size=step_size, goal_bias=goal_bias,
                                goal_tolerance=goal_tolerance, max_iterations=max_iterations,
                                smoothing_attempts=smoothing_attempts))


def save(result, directory):
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'results.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
