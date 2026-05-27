from setuptools import find_packages, setup

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member1',
    maintainer_email='m1@example.com',
    description='Vehicle Control Node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_control_node = vehicle_control.vehicle_control_node:main',
        ],
    },
)
