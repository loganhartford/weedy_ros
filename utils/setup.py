from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['utils', 'utils.*']),  # Ensure 'utils' is found
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='logan.hartford@outlook.com',
    description='Utility functions for multiple ROS2 packages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'uart = {package_name}.uart:main',
        ],
    },
)
