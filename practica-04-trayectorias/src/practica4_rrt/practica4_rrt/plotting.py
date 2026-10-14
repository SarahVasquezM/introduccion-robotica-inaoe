import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .map_utils import OccupancyMap


def plot(filename):
    filename = Path(filename)
    result = json.loads(filename.read_text())
    grid = OccupancyMap(result['world_config'])
    fig, ax = plt.subplots(figsize=(8, 8))
    rows = [grid.data[i*grid.width:(i+1)*grid.width] for i in range(grid.height)]
    ax.imshow(rows, extent=grid.bounds, origin='lower', cmap='Greys', alpha=0.4, vmin=0, vmax=100)
    for i, parent in enumerate(result['parents']):
        if parent >= 0:
            a, b = result['nodes'][parent], result['nodes'][i]
            ax.plot([a[0], b[0]], [a[1], b[1]], color='skyblue', linewidth=0.5)
    for key, color in [('raw', 'orange'), ('smoothed', 'blue')]:
        points = result[key]
        ax.plot([p[0] for p in points], [p[1] for p in points], '-o', color=color,
                markersize=3, label=key)
    for key, color, label in [('start', 'green', 'A'), ('goal', 'red', 'B')]:
        x, y = result[key]
        ax.scatter(x, y, c=color, s=65)
        ax.annotate(label, (x, y), xytext=(5, 5), textcoords='offset points')
    ax.set(xlabel='x [m]', ylabel='y [m]', title=f"RRT seed {result['seed']} ({result['iterations']} iterations)")
    ax.set_aspect('equal')
    ax.legend()
    fig.tight_layout()
    target = filename.with_name('rrt_map.png')
    fig.savefig(target, dpi=150)
    plt.close(fig)
    return target


def main():
    p = argparse.ArgumentParser()
    p.add_argument('results_json')
    args = p.parse_args()
    print(plot(args.results_json))
