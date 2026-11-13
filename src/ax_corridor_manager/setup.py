from glob import glob

from setuptools import find_packages, setup

setup(name='ax_corridor_manager', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/ax_corridor_manager']),
                  ('share/ax_corridor_manager', ['package.xml', 'README.md']),
                  ('share/ax_corridor_manager/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='freem', maintainer_email='freem@todo.todo',
      description='D3 durable corridor grants and occupancy.',
      entry_points={'console_scripts': ['corridor_manager = ax_corridor_manager.node:main']})
