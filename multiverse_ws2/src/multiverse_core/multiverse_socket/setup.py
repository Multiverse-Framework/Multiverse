import os
import setuptools

package_name = "multiverse_socket"
submodule1 = os.path.join(package_name, "multiverse_ros_node")
submodule2 = os.path.join(submodule1, "multiverse_publishers")
submodule3 = os.path.join(submodule1, "multiverse_subscribers")
submodule4 = os.path.join(submodule1, "multiverse_services")

setuptools.setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodule1, submodule2, submodule3, submodule4],
    package_dir={package_name: os.path.join('src', package_name),
                 submodule1: os.path.join('src', submodule1),
                 submodule2: os.path.join('src', submodule2),
                 submodule3: os.path.join('src', submodule3),
                 submodule4: os.path.join('src', submodule4)},
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"]),
                ("lib/python3.8/site-packages/" + package_name, ["scripts/multiverse_socket_node.py"])],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giang Nguyen",
    maintainer_email="hoanggia@uni-bremen.de",
    description="The multiverse_socket package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["multiverse_socket_node = multiverse_socket.multiverse_socket_node:main"],
    },
)
