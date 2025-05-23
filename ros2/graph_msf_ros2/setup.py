from setuptools import setup
from setuptools import find_packages

package_name = 'graph_msf_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='graph_msf_ros2_py'),
    package_dir={'': 'graph_msf_ros2_py'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Nubert',
    maintainer_email='nubertj@ethz.ch',
    description='State estimation based on factor graphs, utilizing GTSAM functionality',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_pose_files_to_tf_and_odom_bag = graph_msf_ros2_py.replay.manual_pose_files_to_tf_and_odom_bag:main',
            'plot_latest_quantitites_in_folder = graph_msf_ros2_py.plotting.plot_latest_quantitites_in_folder:main',
            'remove_tf_from_bag = graph_msf_ros2_py.bag_filter.remove_tf_from_bag:main',
        ],
    },
)