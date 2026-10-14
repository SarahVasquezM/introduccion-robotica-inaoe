"""Generate genuine planning evidence for all scenarios without ROS/Gazebo."""
import argparse
from pathlib import Path
import yaml
from .map_utils import OccupancyMap
from .evidence import plan, save
from .plotting import plot


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True)
    p.add_argument('--scenarios', required=True)
    p.add_argument('--output', default=str(Path.home()/'ros2_ws/results/practica4/offline'))
    args = p.parse_args()
    grid = OccupancyMap.load(args.config)
    scenarios = yaml.safe_load(Path(args.scenarios).read_text())['scenarios']
    for name, scenario in scenarios.items():
        result = plan(grid, scenario['start'], scenario['goal'], scenario['total_time'], scenario['seed'])
        directory = Path(args.output)/name
        save(result, directory)
        plot(directory/'results.json')
        print(f"{name}: {result['iterations']} iterations, "
              f"{result['raw_length']:.3f} -> {result['smoothed_length']:.3f} m, "
              f"nominal {sum(result['segment_times']):.3f} s; flight not executed")


if __name__ == '__main__':
    main()
