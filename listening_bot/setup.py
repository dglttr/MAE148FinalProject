from setuptools import setup

package_name = 'listening_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Glatter, Nick Ji, Shivharsh Kand, Johnny Li',
    maintainer_email='dglatter@ucsd.edu',
    description='Understands verbal commands and sends them to robot.',
    license='MIT',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = listening_bot.publisher_laptop:main',
            'subscriber = listening_bot.subscriber_jetson:main',
            'actuate = listening_bot.hardware:main'
        ],
    },
)
