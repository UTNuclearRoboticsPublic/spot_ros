import os
from glob import glob
from setuptools import setup

package_name = 'spot_driver'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dave Niewinski',
    author_email='dniewinski@clearpathrobotics.com',
    maintainer='Austin Deric',
    maintainer_email='Austin.Deric@gmail.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Proprietary',
        'Programming Language :: Python',
    ],
    description='The spot_driver package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = spot_driver.spot_driver:main',
        ],
    },
)