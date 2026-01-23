import os
from glob import glob

from setuptools import find_packages, setup

package_name = "wam_xbox"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["xbox"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="coledewis@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "xbox_node = wam_xbox.wam_xbox_control:main",
        ],
    },
)
