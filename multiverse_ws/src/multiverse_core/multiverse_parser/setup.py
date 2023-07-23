from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mjcf_to_usd","usd_to_urdf", "sdf_to_urdf"],
    package_dir={
        "mjcf_to_usd": "mjcf_to_usd/scripts/mjcf_to_usd",
        "usd_to_urdf": "usd_to_urdf/scripts/usd_to_urdf",
        "sdf_to_urdf": "sdf_to_urdf/scripts/sdf_to_urdf",
    },
)

setup(**d)
