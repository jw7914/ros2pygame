from setuptools import find_packages, setup

package_name = 'pygame_test'

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
    maintainer='jason',
    maintainer_email='jason@todo.todo',
    description='A package to test joystick input and visualize control states using Tkinter.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = pygame_test.test_node:main",
            "keyboard_publisher = pygame_test.publisher:main",
            "keyboard_listener = pygame_test.listener:main",
            "joystick_node = pygame_test.joystickTester:main"
        ],
    },
)
