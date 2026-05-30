from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_followgap_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='melgui-gei',
    maintainer_email='melgui-gei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'break_ttc_directional = car_followgap_pkg.break_ttc_directional:main',
            'rc_gap_logger = car_followgap_pkg.rc_gap_logger:main',
            'rc_controller = car_followgap_pkg.rc_controller:main',
        ],
    },
)
