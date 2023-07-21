from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["sdf_to_urdf"], package_dir={"": "sdf_to_urdf/scripts"})

setup(**d)
