from setuptools import setup

package_name = 'customize_local_planner'

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
    maintainer='Matthew Lauriault',
    maintainer_email='mlauriault03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localPlanner = customize_local_planner.local_planner2:main',
            'pidTuning = customize_local_planner.pid_tuning_tool:main',
            'phaseOne = customize_local_planner.phase_one_demo:main',
            'gps_plotter = customize_local_planner.gps_plotter:main',
            'odom_plotter = customize_local_planner.odom_plotter:main'
        ],
    },
)
