from setuptools import find_packages, setup

setup(name='ax_grasp_planner', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/ax_grasp_planner']),
                  ('share/ax_grasp_planner', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='freem', maintainer_email='freem@todo.todo',
      description='C3 conservative rule-based suction candidate generation.',
      entry_points={'console_scripts': ['candidate_publisher = ax_grasp_planner.publisher:main']})
