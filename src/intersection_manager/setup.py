from setuptools import find_packages, setup
package_name = 'intersection_manager'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Member4', maintainer_email='m4@example.com',
    description='Central Intersection Manager', license='Apache-2.0',
    entry_points={'console_scripts': [
        'intersection_manager_node = intersection_manager.intersection_manager_node:main',
    ]},
)
