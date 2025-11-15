from setuptools import setup

package_name = 'orion_flight'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Fehn',
    maintainer_email='joshua.fehn@tum.de',
    description='Orion autonomous drone flight control package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_trajectory_node.py = orion_flight.circle_trajectory_node:main',
        ],
    },
)
