from setuptools import find_packages, setup

package_name = 'bumperbot_py_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pulak',
    maintainer_email='pulaktula@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_py_example.simple_publisher:main',
            'simple_subscriber = bumperbot_py_example.simple_subscriber:main',
            'simple_parameter = bumperbot_py_example.simple_parameter:main',
            'simple_turtlesim_kinematics = bumperbot_py_example.simple_turtlesim_kinematics:main',
            'simple_service_server = bumperbot_py_example.simple_service_server:main',
            'simple_service_client = bumperbot_py_example.simple_service_client:main',
            'simple_lifecycle_node = bumperbot_py_example.simple_lifecycle_node:main',
            'simple_action_server = bumperbot_py_example.simple_action_server:main',
            'simple_action_client = bumperbot_py_example.simple_action_client:main',
            'simple_qos_publisher = bumperbot_py_example.simple_qos_pub:main',
            'simple_qos_subscriber = bumperbot_py_example.simple_qos_sub:main',
        ],
    },
)
