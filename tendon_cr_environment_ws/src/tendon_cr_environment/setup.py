from setuptools import find_packages, setup

package_name = 'tendon_cr_environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_tcre.launch.py']), # Control launch file
        ('share/' + package_name + '/launch', ['launch/manual_tcre.launch.py']),  # Manual launch file
        (f'share/{package_name}/images', ['tendon_cr_environment/images/global_reference_frame.png']), # Global frame reference image
        (f'share/{package_name}/images', ['tendon_cr_environment/images/local_reference_frame.png']),  # Local frame reference image

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabotuzl',
    maintainer_email='gabotuzlaci@gmail.com',
    description='ROS2 environment for the simulation and control of a tendon-driven continuum robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simulator_node = tendon_cr_environment.simulator_node:main",
            "visualizer_node = tendon_cr_environment.visualizer_node:main",
            "gui_control_node = tendon_cr_environment.GUI_control:main",
            "gui_manual_node = tendon_cr_environment.GUI_manual:main",
            "controller_node = tendon_cr_environment.controller_node:main",
            "gain_searcher_node = tendon_cr_environment.gain_searcher_node:main"
        ],
    },
)
