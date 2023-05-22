from setuptools import setup

package_name = 'ROS_2_SERIAL'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iljujjang',
    maintainer_email='iljujjang@naver.com',
    description='serial connection on ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = ROS_2_SERIAL.serial_gui:main',
        ],
    },
)
