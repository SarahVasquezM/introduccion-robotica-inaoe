from glob import glob
from os.path import isfile
from setuptools import setup

name = 'practica4_rrt'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                ('share/' + name, ['package.xml', 'README.md'])] +
               [('share/' + name + '/' + folder, [f for f in glob(folder + '/*') if isfile(f)])
                for folder in ('config', 'worlds', 'launch', 'scripts', 'docs', 'rviz')],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='sarah', maintainer_email='sarah@example.com',
    description='Reproducible XY RRT navigation using the existing PID action',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'rrt_navigation = practica4_rrt.rrt_navigation_node:main',
        'plan_scenarios = practica4_rrt.offline:main',
        'plot_rrt_results = practica4_rrt.plotting:main',
        'generate_world = practica4_rrt.world_generator:main']},
)
