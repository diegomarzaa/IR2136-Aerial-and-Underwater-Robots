from glob import glob
from setuptools import find_packages, setup

package_name = 'uw_zed_mock'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='diego@example.com',
    description='Stereo folder publisher that mimics ZED ROS topics.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'stereo_image_folder_publisher = uw_zed_mock.stereo_image_folder_publisher:main',
        ],
    },
)
