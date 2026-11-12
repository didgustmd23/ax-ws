from glob import glob
from setuptools import find_packages, setup

setup(
    name='ax_environment', version='0.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ax_environment']),
        ('share/ax_environment', ['package.xml', 'README.md']),
        ('share/ax_environment/config', glob('config/*.json')),
        ('share/ax_environment/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    maintainer='freem', maintainer_email='freem@todo.todo',
    description='B3 region definitions, conservative geometry checks and corridor observation publisher.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['check_regions = ax_environment.regions:main',
                                    'corridor_observer = ax_environment.corridor_observer:main']},
)
