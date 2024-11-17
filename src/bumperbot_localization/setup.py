from setuptools import find_packages, setup

package_name = 'bumperbot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/local_localization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/global_localization.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
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
            'kalman_filter = bumperbot_localization.kalman_filter:main',
            'imu_republish = bumperbot_localization.imu_republish:main',
            'odometry_motion_model = bumperbot_localization.odometry_motion_model:main'
        ],
    },
    scripts=['bumperbot_localization/imu_republish.py']
)
