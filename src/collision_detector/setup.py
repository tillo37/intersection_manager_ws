from setuptools import find_packages, setup

package_name = 'collision_detector'

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
    maintainer='Member7',
    maintainer_email='m7@example.com',
    description='Collision Detector Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collision_detector_node = collision_detector.collision_detector_node:main',
        ],
    },
)
