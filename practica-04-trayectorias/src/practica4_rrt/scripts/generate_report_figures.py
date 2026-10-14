#!/usr/bin/env python3
"""Generate publication figures and metrics from a genuine Gazebo flight."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-practica4')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from practica4_rrt.map_utils import OccupancyMap


def save_figure(fig, output, name):
    fig.savefig(output/f'{name}.pdf', bbox_inches='tight')
    fig.savefig(output/f'{name}.png', dpi=300, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads((args.run_directory/'results.json').read_text())
    rows = list(csv.DictReader((args.run_directory/'flight.csv').open()))
    if not rows:
        raise ValueError('flight.csv is empty')
    args.output.mkdir(parents=True, exist_ok=True)

    t = np.array([float(row['wall_elapsed']) for row in rows])
    actual = np.array([[float(row[key]) for key in ('x', 'y', 'z')]
                       for row in rows])
    reference = np.array([[float(row[key]) for key in ('ref_x', 'ref_y', 'ref_z')]
                          for row in rows])
    errors = np.array([[float(row[key]) for key in ('err_x', 'err_y', 'err_z')]
                       for row in rows])
    rmse = np.sqrt(np.mean(errors**2, axis=0))
    maximum = np.max(np.abs(errors), axis=0)
    xy_norm = np.linalg.norm(errors[:, :2], axis=1)
    vector_norm = np.linalg.norm(errors, axis=1)

    plt.rcParams.update({
        'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8,
        'axes.titlesize': 8, 'legend.fontsize': 7, 'xtick.labelsize': 7,
        'ytick.labelsize': 7, 'lines.linewidth': 1.0,
        'pdf.fonttype': 42, 'ps.fonttype': 42,
    })

    grid = OccupancyMap(result['world_config'])
    cells = np.array(grid.data).reshape(grid.height, grid.width)
    fig, ax = plt.subplots(figsize=(3.5, 3.4))
    ax.imshow(cells, extent=grid.bounds, origin='lower', cmap='Greys', alpha=0.45,
              vmin=0, vmax=100, interpolation='nearest')
    raw = np.asarray(result['raw'])
    smooth = np.asarray(result['smoothed'])
    ax.plot(raw[:, 0], raw[:, 1], color='#d97706', linestyle=':', marker='.',
            markersize=2.5, label='Raw RRT path')
    ax.plot(smooth[:, 0], smooth[:, 1], color='#2563eb', linestyle='--',
            label='Collision-checked smooth reference')
    ax.plot(actual[:, 0], actual[:, 1], color='#047857', label='Measured trajectory')
    ax.scatter(*result['start'], c='#16a34a', s=30, zorder=5, label='Start A')
    ax.scatter(*result['goal'], c='#dc2626', s=30, zorder=5, label='Goal B')
    ax.set(xlabel=r'$x$ position (m)', ylabel=r'$y$ position (m)',
           xlim=(-1, 7), ylim=(-5.5, 1.0))
    ax.set_xticks(np.arange(-1, 7.1, 1.0))
    ax.set_yticks(np.arange(-5, 1.1, 1.0))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linewidth=0.35, alpha=0.5)
    ax.legend(loc='lower center', ncol=2, fontsize=6)
    save_figure(fig, args.output, 'trajectory_tracking')

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.8), sharex=True)
    colors = ('#b91c1c', '#1d4ed8', '#047857')
    labels = (r'$e_x$', r'$e_y$', r'$e_z$')
    limits = ((-0.16, 0.16), (-0.16, 0.16), (-0.012, 0.012))
    ticks = (np.arange(-0.16, 0.161, 0.08), np.arange(-0.16, 0.161, 0.08),
             np.arange(-0.012, 0.0121, 0.006))
    for index, ax in enumerate(axes):
        ax.plot(t, errors[:, index], color=colors[index],
                label=(f'{labels[index]}: RMSE={rmse[index]:.4f} m, '
                       f'max={maximum[index]:.4f} m'))
        ax.axhline(0.0, color='black', linewidth=0.55)
        ax.set_ylabel(f'{labels[index]} (m)')
        ax.set_ylim(*limits[index])
        ax.set_yticks(ticks[index])
        ax.set_xlim(0.0, result['requested_time'])
        ax.grid(True, linewidth=0.35, alpha=0.55)
        ax.legend(loc='upper right')
    axes[-1].set_xlabel('Elapsed wall time (s)')
    axes[-1].set_xticks(np.arange(0, result['requested_time']+0.1, 2.0))
    save_figure(fig, args.output, 'tracking_errors_xyz')

    reference_velocity = np.gradient(reference, t, axis=0)
    actual_velocity = np.gradient(actual, t, axis=0)
    reference_speed = np.linalg.norm(reference_velocity, axis=1)
    actual_speed = np.linalg.norm(actual_velocity, axis=1)
    window = 31
    kernel = np.ones(window)/window
    reference_speed_filtered = np.convolve(reference_speed, kernel, mode='same')
    actual_speed_filtered = np.convolve(actual_speed, kernel, mode='same')
    peak_reference_theory = 1.5*result['spatial_length']/result['requested_time']
    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    ax.plot(t, reference_speed_filtered, color='#2563eb', label='Reference speed')
    ax.plot(t, actual_speed_filtered, color='#047857', alpha=0.9,
            label='Measured speed (31-sample moving average)')
    ax.axhline(peak_reference_theory, color='#7c3aed', linestyle=':',
               label=f'Theoretical cubic peak: {peak_reference_theory:.3f} m/s')
    ax.set(xlabel='Elapsed wall time (s)', ylabel='Speed magnitude (m/s)',
           xlim=(0.0, result['requested_time']), ylim=(0.0, 0.9))
    ax.set_xticks(np.arange(0, result['requested_time']+0.1, 2.0))
    ax.set_yticks(np.arange(0, 0.91, 0.15))
    ax.grid(True, linewidth=0.35, alpha=0.55)
    ax.legend(loc='upper right')
    save_figure(fig, args.output, 'speed_profile')

    valid_speed = actual_speed_filtered[window//2:-(window//2)]
    metrics = {
        'source_run': str(args.run_directory),
        'samples': len(rows),
        'sample_period_mean_s': float((t[-1]-t[0])/(len(t)-1)),
        'requested_time_s': result['requested_time'],
        'wall_time_s': result['real_time'],
        'timing_error_s': result['real_time']-result['requested_time'],
        'timing_error_percent': 100.0*(result['real_time']-result['requested_time']) /
                                result['requested_time'],
        'planning_time_s': result['planning_wall_seconds'],
        'iterations': result['iterations'],
        'tree_nodes': len(result['nodes']),
        'raw_length_m': result['raw_length'],
        'smooth_length_m': result['smoothed_length'],
        'spatial_length_m': result['spatial_length'],
        'minimum_time_s': result['minimum_time'],
        'shortcut_waypoints': len(result.get('shortcut_smoothed', [])),
        'rounded_samples': len(result['smoothed']),
        'rmse_x_m': float(rmse[0]), 'rmse_y_m': float(rmse[1]),
        'rmse_z_m': float(rmse[2]),
        'rmse_vector_m': float(math.sqrt(np.mean(vector_norm**2))),
        'max_abs_x_m': float(maximum[0]), 'max_abs_y_m': float(maximum[1]),
        'max_abs_z_m': float(maximum[2]),
        'max_xy_norm_m': float(np.max(xy_norm)),
        'final_position_norm_m': result['final_error']['norm'],
        'final_yaw_error_rad': abs(result['final_error']['yaw']),
        'peak_reference_speed_theory_mps': peak_reference_theory,
        'peak_measured_speed_filtered_mps': float(np.max(valid_speed)),
    }
    (args.output/'metrics.json').write_text(json.dumps(metrics, indent=2))
    macros = {
        'Samples': f"{metrics['samples']}",
        'RequestedTime': f"{metrics['requested_time_s']:.3f}",
        'WallTime': f"{metrics['wall_time_s']:.3f}",
        'TimingError': f"{metrics['timing_error_s']:.3f}",
        'TimingPercent': f"{metrics['timing_error_percent']:.3f}",
        'PlanningTime': f"{metrics['planning_time_s']:.3f}",
        'Iterations': f"{metrics['iterations']}",
        'TreeNodes': f"{metrics['tree_nodes']}",
        'RawLength': f"{metrics['raw_length_m']:.3f}",
        'SmoothLength': f"{metrics['smooth_length_m']:.3f}",
        'MinimumTime': f"{metrics['minimum_time_s']:.3f}",
        'RmseX': f"{metrics['rmse_x_m']:.4f}",
        'RmseY': f"{metrics['rmse_y_m']:.4f}",
        'RmseZ': f"{metrics['rmse_z_m']:.4f}",
        'RmseVector': f"{metrics['rmse_vector_m']:.4f}",
        'MaxX': f"{metrics['max_abs_x_m']:.4f}",
        'MaxY': f"{metrics['max_abs_y_m']:.4f}",
        'MaxZ': f"{metrics['max_abs_z_m']:.4f}",
        'MaxXY': f"{metrics['max_xy_norm_m']:.4f}",
        'FinalError': f"{metrics['final_position_norm_m']:.4f}",
        'FinalYaw': f"{metrics['final_yaw_error_rad']:.6f}",
        'PeakReferenceSpeed': f"{metrics['peak_reference_speed_theory_mps']:.3f}",
        'PeakMeasuredSpeed': f"{metrics['peak_measured_speed_filtered_mps']:.3f}",
    }
    lines = [f'\\newcommand{{\\{name}}}{{{value}}}' for name, value in macros.items()]
    (args.output/'metrics.tex').write_text('\n'.join(lines)+'\n')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
