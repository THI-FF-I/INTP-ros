from setuptools import setup
import os

import setuptools
import warnings
warnings.simplefilter("ignore", setuptools.SetuptoolsDeprecationWarning)

package_name = 'talker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share/' + package_name), ['package.xml', os.path.join('config', 'parameters.yaml'),
                                   os.path.join('launch', 'talker_launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-dev',
    maintainer_email='52259564+J0las@users.noreply.github.com',
    description='Simple talker node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = talker.talker:main',
        ],
    },
)
