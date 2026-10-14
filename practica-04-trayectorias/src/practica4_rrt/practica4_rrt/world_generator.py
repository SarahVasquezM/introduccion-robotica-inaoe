"""Generate physical obstacle geometry from the same YAML as the planner."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from .map_utils import OccupancyMap


def generate(config, target):
    grid = OccupancyMap.load(config)
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name='rrt_obstacles')
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    ET.SubElement(world, 'plugin', name='gazebo_ros_state', filename='libgazebo_ros_state.so')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    ground = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(ground, 'static').text = 'true'
    link = ET.SubElement(ground, 'link', name='link')
    for kind in ('collision', 'visual'):
        plane = ET.SubElement(ET.SubElement(ET.SubElement(link, kind, name=kind), 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = '100 100'
    for o in grid.config['obstacles']:
        model = ET.SubElement(world, 'model', name=o['name'])
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f"{o['x']} {o['y']} {o['height']/2} 0 0 0"
        link = ET.SubElement(model, 'link', name='link')
        for kind in ('collision', 'visual'):
            shape = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(shape, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = f"{o['width']} {o['depth']} {o['height']}"
            if kind == 'visual':
                ET.SubElement(ET.SubElement(shape, 'material'), 'diffuse').text = '0.8 0.3 0.15 1'
    ET.indent(sdf)
    ET.ElementTree(sdf).write(target, encoding='unicode', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config')
    parser.add_argument('world')
    args = parser.parse_args()
    generate(args.config, Path(args.world))


if __name__ == '__main__':
    main()
