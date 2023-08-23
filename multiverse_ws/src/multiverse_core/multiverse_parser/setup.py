from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["sdf_to_urdf", "urdf_to_fbx"],
    package_dir={
        "sdf_to_urdf": "sdf_to_urdf/scripts/sdf_to_urdf",
        "urdf_to_fbx": "urdf_to_fbx/scripts/urdf_to_fbx",
    },
)

setup(**d)
