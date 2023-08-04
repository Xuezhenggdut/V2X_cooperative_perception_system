from setuptools import setup
import os
from glob import glob

package_name = 'point_cloud_infer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # During installation, we need to copy the launch files
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liao canliang',
    maintainer_email='canliang148@foxmail.com',
    description='Point cloud 3D object detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = point_cloud_infer.point_cloud_publisher:main',
            'visualization_subscriber = point_cloud_infer.point_cloud_visualization:main',
            'bbox_3d_visualization = point_cloud_infer.bounding_box_3d_visualization:main',
            'bounding_box_client = point_cloud_infer.bounding_box_client:main',
            'bounding_box_server = point_cloud_infer.bounding_box_server:main',
            'bounding_box_listener_publisher = point_cloud_infer.bounding_box_listener:main',
        ],
    },
)
