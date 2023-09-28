import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'spot_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dave Niewinski',
    author_email='dniewinski@clearpathrobotics.com',
    maintainer='Alex Navarro',
    maintainer_email='alexnavtt@utexas.edu',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Proprietary',
        'Programming Language :: Python',
    ],
    description='The spot_bringup package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_combined = spot_bringup.driver_combined:main',
        ],
    },
)