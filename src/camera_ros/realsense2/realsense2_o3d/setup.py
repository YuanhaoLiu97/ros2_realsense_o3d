from setuptools import setup

package_name = 'realsense2_o3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['realsense2_o3d/launch/rs_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sjl',
    maintainer_email='sjl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_realsense2_o3d = realsense2_o3d.node_realsense2_o3d:main'
        ],
    },
)
