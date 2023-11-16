from setuptools import setup

package_name = 'multiverse_socket'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giang Nguyen',
    maintainer_email='hoanggia@uni-bremen.de',
    description='The multiverse_socket package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiverse_socket_node = multiverse_socket.multiverse_socket_node:main'
        ],
    },
)
