from setuptools import setup
from glob import glob
import os

package_name = 'spot_description'

def list_dir_files(dir: str):
    mesh_files = []
    for (dirpath, _, filenames) in os.walk(dir):
        files = [os.path.join(dirpath, f) for f in filenames]
        mesh_files.append((os.path.join('share', package_name, dirpath), files))
    return mesh_files

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *list_dir_files('meshes'),
        *list_dir_files('urdf')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Navarro',
    maintainer_email='alexnavtt@utexas.edu',
    description='Spot description package',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
    ],
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
