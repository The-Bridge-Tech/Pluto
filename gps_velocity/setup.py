from setuptools import setup

<<<<<<< HEAD:customize_local_planner/setup.py
package_name = 'customize_local_planner'
=======
package_name = 'gps_velocity'
>>>>>>> devel:gps_velocity/setup.py

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
    maintainer='shouyu',
    maintainer_email='joeldushouyu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD:customize_local_planner/setup.py
            'localMovingPlanner = customize_local_planner.local_planner:main',
=======
            "gps_velocity=gps_velocity.velocityCalculation:main"
>>>>>>> devel:gps_velocity/setup.py
        ],
    },
)
