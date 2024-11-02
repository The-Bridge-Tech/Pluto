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
            'local_planner = customize_local_planner.local_planner2:main',
            'phase_one_demo = customize_local_planner.phase_one_demo:main',
        ],
    },
)
