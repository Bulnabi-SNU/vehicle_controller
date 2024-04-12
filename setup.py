from setuptools import setup

package_name = 'vehicle_controller'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juyong Shin',
    maintainer_email='juyong3393@snu.ac.kr',
    description='Bulnabi 2024 vehicle controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller = vehicle_controller.vehicle_controller:main',
        ],
    },
)
