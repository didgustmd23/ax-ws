from setuptools import find_packages, setup


package_name = "ax_d4"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "rfc8785"],
    zip_safe=True,
    maintainer="freem",
    maintainer_email="freem@todo.todo",
    description="D4 RobotTaskStatus validation and durable ledger.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["d4_main = ax_d4.d4_main:main"]},
)
