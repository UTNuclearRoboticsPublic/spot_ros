import os
from glob import glob
from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'spot_simulation'

generate_parameter_module(
  "simulation_parameters", # python module name for parameter library
  f"{package_name}/simulation_parameters.yaml", # path to input yaml file
)

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'mesh'), glob('mesh/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alex Navarro',
    author_email='alexnavtt@utexas.edu',
    maintainer='Alex Navarro',
    maintainer_email='alexnavtt@utexas.edu',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Proprietary',
        'Programming Language :: Python',
    ],
    description='A simulation package for Spot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'simulation = {package_name}.simulation:main',
            f'simulated_servers = {package_name}.simulated_servers:main'
        ],
    },
)