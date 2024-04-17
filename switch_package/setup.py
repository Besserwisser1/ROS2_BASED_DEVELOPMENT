from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'switch_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib/python3.10/site-packages/switch_package/receiver/'), glob('./switch_package/receiver/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orange',
    maintainer_email='tr4in33.oak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'switch_node = ' + package_name + '.main:main',
        ],
    },
)
