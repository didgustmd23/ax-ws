from glob import glob

from setuptools import find_packages, setup

package_name = 'ax_operations_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web',
         glob('web/*.html') + glob('web/*.css') + glob('web/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yhs',
    maintainer_email='yhs@todo.todo',
    description=('D2 operations console: subscribe-only display relay '
                 'with a GET/HEAD-only web page.'),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'operations_console = ax_operations_console.console_node:main',
        ],
    },
)
