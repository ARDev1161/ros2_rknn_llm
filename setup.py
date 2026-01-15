from setuptools import setup

package_name = 'ros2_rknn_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/system_prompt.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 gateway node for rknn-llm with PDDL goal output.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'ros2_rknn_llm_node = ros2_rknn_llm.node:main',
        ],
    },
)
