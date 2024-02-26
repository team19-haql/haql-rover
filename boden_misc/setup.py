from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'boden_misc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('*.py')),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes', 'dae'), glob('meshes/dae/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deweykai',
    maintainer_email='deweykai5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loader = boden_misc.loader:main',
            'i2c_bus = boden_misc.i2c_bus:main',
            'mock_battery = boden_misc.mock_battery:main',
        ],
    },
)
