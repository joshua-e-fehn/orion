from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orion_flight'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Joshua Fehn',
    maintainer_email='joshua.fehn@tum.de',
    description='Orion autonomous drone flight control package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_trajectory_node = orion_flight.circle_trajectory_node:main',
            # Predictor nodes
            'cv_predictor_node = orion_flight.predictors.cv.cv_predictor_node:main',
            'ca_predictor_node = orion_flight.predictors.ca.ca_predictor_node:main',
            # Planner nodes
            'pp_planner = orion_flight.planners.pp.pp_planner_node:main',
        ],
    },
)
