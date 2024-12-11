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
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
        ('share/' + package_name + '/config', ['config/high_density_preset.json']),
    ],
    install_requires=['setuptools'],
    py_modules=[
        f"{package_name}.moveIt_api",
    ],
    zip_safe=True,
    maintainer='Courtney Smith',
    maintainer_email='courtneysmith2024@u.northwestern.edu',
    description='Facilitates Diffusion Policy on the Franka Emika Panda Robot',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_franka_bridge = action_franka_bridge.action_franka_bridge:main',
            'data_collection = action_franka_bridge.data_collection:main',
            'model_input_publisher = action_franka_bridge.model_input_publisher:main',
            'command_mode = action_franka_bridge.command_mode:main'
        ],
    },
)
