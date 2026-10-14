from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
import yaml
from practica4_rrt.map_utils import OccupancyMap
from practica4_rrt.rrt_planner import RRTPlanner, PlanningError
from practica4_rrt.path_smoothing import smooth_path
from practica4_rrt.trajectory_utils import (
    ArcLengthPath, cubic_reference, minimum_time_for_speed, path_length,
    peak_cubic_speed, rounded_path, segment_times
)
from practica4_rrt.evidence import plan
from practica4_rrt.world_generator import generate

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def grid():
    return OccupancyMap.load(ROOT/'config/world_config.yaml')


def test_world_grid_roundtrip(grid):
    for ix in range(grid.width):
        for iy in (0, grid.height//2, grid.height-1):
            assert grid.world_to_grid(*grid.grid_to_world(ix, iy)) == (ix, iy)
    with pytest.raises(ValueError):
        grid.world_to_grid(grid.bounds[1], 0)
    with pytest.raises(ValueError):
        grid.grid_to_world(-1, 0)
    assert not grid.point_free((float('nan'), 0))


def test_points_edges_and_corner(grid):
    assert grid.point_free((0, 0))
    assert not grid.point_free((3, 0))
    assert not grid.point_free((2, 0))  # inflated obstacle
    assert not grid.point_free((7.9, 0))  # boundary margin
    assert not grid.segment_free((0, 0), (6, 0))
    assert grid.segment_free((0, 0), (0, -2))
    assert OccupancyMap.intersects((0, 1), (1, 0), (0.5, 1, 0.5, 1))
    assert not OccupancyMap.intersects((0, 0), (0, 1), (1, 2, 0, 1))


@pytest.mark.parametrize('a,b', [((-20, 0), (0, 0)), ((0, 0), (3, 0)), ((0, 0), (0, 0))])
def test_invalid_endpoints(grid, a, b):
    with pytest.raises(ValueError):
        RRTPlanner(grid).plan(a, b)


def test_reconstruct(grid):
    planner = RRTPlanner(grid)
    planner.nodes = [(0, 0), (1, 0), (1, 1)]
    planner.parents = [-1, 0, 1]
    assert planner.reconstruct(2) == planner.nodes


@pytest.mark.parametrize('name', ['simple', 'detour', 'alternate'])
def test_scenarios(grid, name):
    s = yaml.safe_load((ROOT/'config/scenarios.yaml').read_text())['scenarios'][name]
    result = plan(grid, s['start'], s['goal'], s['total_time'], s['seed'])
    assert grid.validate_path(result['raw'])
    assert grid.validate_path(result['smoothed'])
    assert result['raw'][0] == tuple(s['start'])
    assert result['raw'][-1] == tuple(s['goal'])
    assert result['smoothed_length'] <= result['raw_length']+1e-10
    assert all(t > 0 for t in result['segment_times'])
    assert sum(result['segment_times']) == pytest.approx(s['total_time'])
    again = plan(grid, s['start'], s['goal'], s['total_time'], s['seed'])
    assert again['raw'] == result['raw']
    if name != 'simple':
        assert not grid.segment_free(s['start'], s['goal'])


def test_no_solution():
    grid = OccupancyMap(dict(bounds=[-2, 2, -2, 2], resolution=0.1,
                             drone_radius=0, margin=0,
                             obstacles=[dict(x=0, y=0, width=0.2, depth=4, height=5)]))
    with pytest.raises(PlanningError, match='No route after'):
        RRTPlanner(grid, max_iterations=100).plan((-1, 0), (1, 0))


def test_times_cubic_and_parameters(grid):
    assert segment_times([(0, 0), (1, 0), (4, 0)], 8) == [2, 6]
    assert cubic_reference((0, 0, 2), (4, 0, 2), 0, 8) == (0, 0, 2)
    assert cubic_reference((0, 0, 2), (4, 0, 2), 8, 8) == (4, 0, 2)
    for value in (0, -1, math.nan):
        with pytest.raises(ValueError):
            segment_times([(0, 0), (1, 0)], value)
    with pytest.raises(ValueError):
        segment_times([(0, 0), (0, 0)], 5)
    with pytest.raises(ValueError):
        RRTPlanner(grid, goal_bias=2)
    with pytest.raises(ValueError):
        smooth_path([(0, 0), (6, 0)], grid)
    assert peak_cubic_speed([(0, 0), (1, 0)], 2.0) == pytest.approx(0.75)
    assert minimum_time_for_speed([(0, 0), (1, 0)], 0.75) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        minimum_time_for_speed([(0, 0), (1, 0)], 0.0)


def test_rounded_path_is_collision_free_and_continuous(grid):
    result = plan(grid, (0, 0), (6, 0), 30.0, 22)
    rounded = rounded_path(result['smoothed'], grid)
    assert grid.validate_path(rounded)
    assert rounded[0] == result['smoothed'][0]
    assert rounded[-1] == result['smoothed'][-1]
    arc = ArcLengthPath(rounded)
    assert arc.length > 0.0
    start, tangent = arc.sample(0.0)
    end, _ = arc.sample(arc.length)
    assert start == rounded[0] and end == pytest.approx(rounded[-1])
    assert math.hypot(*tangent) == pytest.approx(1.0)


def test_generated_world_matches_config(grid, tmp_path):
    target = tmp_path/'world.world'
    generate(ROOT/'config/world_config.yaml', target)
    root = ET.parse(target).getroot()
    for obstacle in grid.config['obstacles']:
        model = root.find(f"world/model[@name='{obstacle['name']}']")
        pose = list(map(float, model.findtext('pose').split()))
        assert pose[:3] == [obstacle['x'], obstacle['y'], obstacle['height']/2]
        for kind in ('visual', 'collision'):
            assert list(map(float, model.findtext(f'link/{kind}/geometry/box/size').split())) == [
                obstacle['width'], obstacle['depth'], obstacle['height']]
    assert target.read_text() == (ROOT/'worlds/rrt_obstacles.world').read_text()
