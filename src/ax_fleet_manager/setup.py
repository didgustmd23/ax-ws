from glob import glob

from setuptools import find_packages, setup

package_name = 'ax_fleet_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yhs',
    maintainer_email='yhs@todo.todo',
    description=('D2 group task step machine, receipt intake and inventory '
                 '(W3-W5 development, mock ports only).'),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'group_dev_publisher = ax_fleet_manager.group_node:main',
            'd2_dev_intake = ax_fleet_manager.intake_node:main',
        ],
    },
)
