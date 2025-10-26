from setuptools import find_packages, setup

package_name = 'adc_pot_publisher'

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
    description='Publishes potentiometer values from ADS7830',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adc_pot_pub_node = adc_pot_publisher.adc_pot_publisher:main',
        ],
    },
)
