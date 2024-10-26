from setuptools import find_packages, setup

package_name = 'camera_package'

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
    maintainer='loganhartford',
    maintainer_email='logan.hartford@outlook.com',
    description='Pi Camera Module 3 Control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'camera = {package_name}.camera:main',
        ],
    },
)
