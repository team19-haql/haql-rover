from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'boden_navigation'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name), ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
data_files.append((os.path.join('share', package_name, 'config'), glob('config/*.yml')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deweykai',
    maintainer_email='deweykai5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_striaght = boden_navigation.calibrate_straight:main',
            'calibrate_box = boden_navigation.calibrate_box:main',
            'gps_wpf = boden_navigation.gps_wpf:main',
        ],
    },
)
