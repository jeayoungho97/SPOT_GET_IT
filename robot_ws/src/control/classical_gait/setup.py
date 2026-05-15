from setuptools import setup
from glob import glob
import os

package_name = 'classical_gait'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY S14P21A206',
    maintainer_email='jeayoungho@gmail.com',
    description='Classical (deterministic) trot gait controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'classical_gait_node = classical_gait.classical_gait_node:main',
        ],
    },
)
