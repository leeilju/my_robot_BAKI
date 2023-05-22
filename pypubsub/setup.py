from setuptools import setup

package_name = 'pypubsub'

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
    maintainer='leeilju', 
    maintainer_email='iljujjang@naver.com', 
    description='minimal pub/sub using rclpy', 
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pypubsub.publisher_member_function:main' ,
            'listnerandtalker2 = pypubsub.subscriber_member_function:main',
            'listener2 = pypubsub.subscriber2:main'
        ],
    },
)
