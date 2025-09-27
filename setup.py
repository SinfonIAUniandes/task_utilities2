from setuptools import find_packages, setup

package_name = 'task_utilities2'

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
    maintainer='sinfonia',
    maintainer_email='sinfonia@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_task_simple = task_utilities2.simple_task_module_example:main',
            'realtime_event = task_utilities2.event_realtime_with_task_module:main',
            'miscellaneous_example = task_utilities2.miscellaneous_functionality_example:main',
            'manipulation_example = task_utilities2.manipulation_example:main',
            'minimal_robot_agent = task_utilities2.minimal_robot_agent_example:main',
        ],
    },
)
