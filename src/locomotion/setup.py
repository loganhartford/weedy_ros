from setuptools import find_packages, setup

package_name = 'locomotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='logan.hartford@outlook.com',
    description='Controls the differential drive system of the WeedWarden Robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'controller = {package_name}.controller:main',
            f'localization = {package_name}.localization:main',
            f'odometry = {package_name}.odometry:main',
            f'bno085_imu = {package_name}.bno085_imu:main',
        ],
    },
)
