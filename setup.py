from setuptools import setup

package_name = 'devkit_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'torch'],
    zip_safe=True,
    maintainer='Christian Diekmann',
    maintainer_email='ChristianDiekmann@aaeon.eu',
    description='YOLO object detection for the AMR development kit',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
'detection_node = devkit_object_detection.detect:main'
        ],
    },
)
