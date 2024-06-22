from setuptools import find_packages, setup

package_name = 'realsense_camera'

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
    description='Package for Intel RealSense Depth Camera 435',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = realsense_camera.CameraNode:main',
        ],
    },
)

