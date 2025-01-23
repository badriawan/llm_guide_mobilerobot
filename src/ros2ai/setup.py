import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ros2ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.py')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mac',
    maintainer_email='yusufbadriawan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_api_server = ros2ai.nav2_api_server:main',
            'Nav2Gpt = ros2ai.nav_gpt:main',
            'Nav2Chatgpt = ros2ai.nav_chatgpt:main',
            'Nav24o = ros2ai.nav_4o:main',
            'Nav2Rag = ros2ai.nav_rag:main',


        ],
    },
)
