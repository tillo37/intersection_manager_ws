from setuptools import find_packages, setup
package_name = 'traffic_light_ctrl'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Member2', maintainer_email='m2@example.com',
    description='Traffic Light Controller', license='Apache-2.0',
    entry_points={'console_scripts': [
        'traffic_light_node = traffic_light_ctrl.traffic_light_node:main',
    ]},
)
