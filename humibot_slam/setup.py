from setuptools import find_packages, setup
from glob import glob

package_name = 'humibot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config/',
            glob('config/*.yaml')),
        (f'share/{package_name}/launch/',
            glob('launch/*launch.[pxy][yma]')),
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
            "set_init_amcl_pose=humibot_slam.set_init_amcl_pose:main"
        ],
    },
)
