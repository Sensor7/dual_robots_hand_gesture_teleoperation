from setuptools import find_packages, setup

package_name = 'hand_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/integrate_servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guanqi Chen',
    maintainer_email='guanqichen707@outlook.com',
    description='TODO: Package description',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_teleop = hand_teleop.hand_teleop:main'
        ],
    },
)
