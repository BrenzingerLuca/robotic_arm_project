from setuptools import find_packages, setup

package_name = 'potentiometer_pub'

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
    maintainer='lucab',
    maintainer_email='luca.brenzinger@gmail.com',
    description='ROS 2 package for publishing potentiometer values via I2C ADC',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'potentiometer_pub_node = potentiometer_pub.potentiometer_pub:main',
        ],
    },
)
