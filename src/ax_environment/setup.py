from glob import glob
from setuptools import find_packages, setup

setup(
    name='ax_environment', version='0.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ax_environment']),
        ('share/ax_environment', ['package.xml', 'README.md']),
        ('share/ax_environment/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    maintainer='freem', maintainer_email='freem@todo.todo',
    description='B3 region definitions and conservative geometry checks.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['check_regions = ax_environment.regions:main']},
)
