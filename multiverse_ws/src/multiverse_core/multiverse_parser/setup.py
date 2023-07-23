from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["usd_to_urdf", "sdf_to_urdf"],
    package_dir={
        "usd_to_urdf": "usd_to_urdf/scripts/usd_to_urdf",
        "sdf_to_urdf": "sdf_to_urdf/scripts/sdf_to_urdf",
    },
)

setup(**d)
