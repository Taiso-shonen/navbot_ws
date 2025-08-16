from setuptools import find_packages, setup

package_name = 'my_navbot_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
        'config/waypoints.yaml'
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ts',
    maintainer_email='saeedharb00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_map_saver = my_navbot_tools.my_map_saver:main',
            'nav2_mapping = my_navbot_tools.nav2_mapping:main',
            'nav2_mapping_cluster = my_navbot_tools.nav2_mapping_cluster:main',
            'nav2_mapping_cluster_beta = my_navbot_tools.nav2_mapping_cluster_beta:main',
            'nav_to_pose_client = my_navbot_tools.nav_to_pose_client:main',
            'current_pose = my_navbot_tools.current_position:main',
            'path_validation = my_navbot_tools.path_validation:main',
            'object_detector = my_navbot_tools.object_detector:main',
            'object_tracker = my_navbot_tools.object_tracker:main',
            'waypoints_patrol = my_navbot_tools.waypoints_patrol:main'
        ],
    },
)
