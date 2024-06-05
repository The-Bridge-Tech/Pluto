from setuptools import setup

package_name = 'splunk_logger'

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
    description='Send data to splunk server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'splunk_logger = splunk_logger.logger:main',
        ],
    },
)
