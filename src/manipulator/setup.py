from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*')),
        (os.path.join('share',package_name),glob('urdf/*')),
        (os.path.join('share',package_name),glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swaraj',
    maintainer_email='swaraj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_fk = manipulator.controller_fk:main",
            "controller_ik = manipulator.controller_ik:main",
            'controller=manipulator.controller:main',
        ],
    },
)
