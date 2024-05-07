from setuptools import find_packages, setup
from glob import glob

package_name = 'humibot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch/',
            glob('launch/*launch.[pxy][yma]'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ervinpicardal',
    maintainer_email='ervinjohnpicardal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dehumidifier_service_node=humibot_hardware.dehumidifier_service_node:main",
            "dht11_node=humibot_hardware.dht11_node:main",
            "dht11_service=humibot_hardware.dht11_service:main",
            "WSService=humibot_hardware.WSService:main",
            "water_lvl_sensor_node=humibot_hardware.water_lvl_sensor_node:main"
        ],
    },
)
