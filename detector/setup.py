from setuptools import setup

import os
from glob import glob
from urllib.request import urlretrieve
from setuptools import find_packages

package_name = 'detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')), 
        (os.path.join('lib/python3.10/site-packages/detector/rknn/new/'), glob('./detector/rknn/new/*.rknn')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tr4in33',
    maintainer_email='tr4in33.oak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'detector_node = '+package_name+'.main:ros_main', 
        ],
    },
)
