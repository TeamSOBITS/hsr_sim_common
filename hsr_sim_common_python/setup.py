from setuptools import find_packages, setup

package_name = 'hsr_sim_common_python'

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
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = hsr_sim_common_python.joint_controller:main',
            'grasp_obj_by_frame_hsr = hsr_sim_common_python.grasp_obj_by_frame_hsr:main',
            'odom_base_controller = hsr_sim_common_python.odom_base_controller:main',
            'pointing_node = hsr_sim_common_python.pointing:main',
            'potential = hsr_sim_common_python.potential:main',
            'test_grasp_node = hsr_sim_common_python.test_grasp:main',
            'sigverse_tf_time_changer = hsr_sim_common_python.tf_time_change:main',
        ],
    },
)
