from setuptools import find_packages, setup

package_name = 'pygame_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Exclude the 'test' directory if not needed
    data_files=[
        # ROS 2 resource index for packages
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # Include package.xml for ROS 2
    ],
    install_requires=['setuptools'],  # Dependencies for setup
    zip_safe=True,
    maintainer='jason',
    maintainer_email='jason@todo.todo',  # Replace with actual maintainer info
    description='A package to test joystick input and visualize control states using Tkinter.',
    license='MIT',  # Include license type (if applicable)
    tests_require=['pytest'],  # Required for testing
    entry_points={
        'console_scripts': [
            # List of ROS 2 nodes that can be executed via ros2 run
            "test_node = pygame_test.test_node:main",
            "keyboard_publisher = pygame_test.publisher:main",
            "keyboard_listener = pygame_test.listener:main",
            "joystick_visualizer = pygame_test.joystickVisualizer:main",
            "joystick = pygame_test.joyTest:main",
        ],
    },
)

