from setuptools import find_packages, setup
package_name = 'cli_dashboard'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'click'],
    zip_safe=True,
    maintainer='Member5', maintainer_email='m5@example.com',
    description='CLI Dashboard', license='Apache-2.0',
    entry_points={'console_scripts': [
        'cli_dashboard = cli_dashboard.cli_dashboard:main',
    ]},
)
