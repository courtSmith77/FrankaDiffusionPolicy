from setuptools import find_packages, setup

package_name = 'action_franka_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/integrate_servo.launch.py']),
        ('share/' + package_name + '/config', ['config/integrate.rviz']),
        ('share/' + package_name + '/config', ['config/integrate_servo.rviz']),
        ('share/' + package_name + '/config', ['config/2vids.rviz']),
        ('share/' + package_name + '/config', ['config/multi_img.rviz']),
    ],
    install_requires=['setuptools'],
    py_modules=[
        f"{package_name}.moveIt_api",
    ],
    zip_safe=True,
    maintainer='Graham Clifford',
    maintainer_email='gclifford@u.northwestern.edu',
    description='TODO: Package description',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_franka_bridge = action_franka_bridge.action_franka_bridge:main',
            'data_collection = action_franka_bridge.data_collection:main',
            'model_input_publisher = action_franka_bridge.model_input_publisher:main'
        ],
    },
)
