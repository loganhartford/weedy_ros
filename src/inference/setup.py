from setuptools import find_packages, setup

package_name = 'inference'

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
    description='Runs YOLO inference on images to determine the keypoints of dandelions.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'inference = {package_name}.inference:main',
        ],
    },
)
