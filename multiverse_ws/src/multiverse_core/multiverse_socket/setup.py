import os
import setuptools

package_name = os.path.join("multiverse_socket")

setuptools.setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={package_name: os.path.join("src", package_name)},
    install_requires=["setuptools"],
    maintainer="Giang Nguyen",
    maintainer_email="hoanggia@uni-bremen.de",
    description="The multiverse_socket package",
    license="MIT",
    tests_require=["pytest"],
)
