from setuptools import find_packages, setup
package_name = 'pedestrian_sim'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Member3', maintainer_email='m3@example.com',
    description='Pedestrian Simulation', license='Apache-2.0',
    entry_points={'console_scripts': [
        'pedestrian_sim_node = pedestrian_sim.pedestrian_sim_node:main',
    ]},
)
