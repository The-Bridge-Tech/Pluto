from setuptools import find_packages, setup

package_name = 'simulator'

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
    maintainer='Matthew Lauriault',
    maintainer_email='mlauriault03@gmail.com',
    description='Test Simulator and Analyzer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'logic_tester = simulator.logic_tester:main',
                'analyzer = simulator.analyzer:main',
                'pwm_plotter = customize_local_planner.pwm_plotter:main'
        ],
    },
)
