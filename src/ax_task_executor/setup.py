from setuptools import find_packages, setup

package_name = 'ax_task_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='freem',
    maintainer_email='freem@todo.todo',
    description=('A3 mock local task Action server, development-policy lease check '
                 'server and Gazebo result display.'),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mock_executor = ax_task_executor.mock_executor:main',
            'gazebo_mock_display = ax_task_executor.gazebo_mock_display:main',
            'lease_check = ax_task_executor.lease_check:main',
        ],
    },
)
