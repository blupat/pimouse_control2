from glob import glob
from setuptools import setup

package_name = 'pimouse_control2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('launch/*.yaml')),
        ('share/' + package_name + '/contents', ['contents/index.html', 'contents/main.js']),
        ('share/' + package_name + '/contents/css', glob('contents/css/*.css')),
        ('share/' + package_name + '/contents/css', glob('contents/css/*.map')),
        ('share/' + package_name + '/contents/js', glob('contents/js/*.js')),
        ('share/' + package_name + '/contents/js', glob('contents/js/*.map')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blupat',
    maintainer_email='oosmyss@gmail.com',
    description='The pimouse_control2 package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pimouse_control_node = ' + package_name + '.pimouse_control_node:main',
            'webserver = ' + package_name + '.webserver:main',
        ],
    },
)
