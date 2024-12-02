from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rviz2_visuals'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arrow_random = rviz2_visuals.arrow_random:main',
            'circle_wave = rviz2_visuals.circle_wave:main',
            'circle_path = rviz2_visuals.circle_path:main',
            'image_publisher = rviz2_visuals.image_publisher:main',
        ],
    },
)
