from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[

        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'description'), glob('description/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlios',
    maintainer_email='carlioseryan20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_laser_scan = articubot_one.depth_to_laser_scan:main',
        ],
    },
)
