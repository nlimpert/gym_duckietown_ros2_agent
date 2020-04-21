from setuptools import setup

package_name = 'gym_duckietown_ros2_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Limpert',
    maintainer_email='limpert@fh-aachen.de',
    description='ROS2 agent for the Duckietown gym simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    py_modules=['env'],
    entry_points={
        'console_scripts': [
                'rosagent = gym_duckietown_ros2_agent.rosagent:main',
        ],
#        'dependencies': [
#                'env = gym_duckietown_ros2_agent.env:launch_env',
#        ],
    },
)
