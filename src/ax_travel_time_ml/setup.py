from setuptools import find_packages, setup

package_name = 'ax_travel_time_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yhs',
    maintainer_email='yhs@todo.todo',
    description='D ETA baseline and XGBoost training/evaluation (W7 development, offline).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ax_eta_train = ax_travel_time_ml.cli:train_entry',
            'ax_eta_evaluate = ax_travel_time_ml.cli:evaluate_entry',
            'ax_eta_synth = ax_travel_time_ml.cli:synth_entry',
        ],
    },
)
