from setuptools import setup

package_name = 'serial_motor_demo'

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
    maintainer='valiantlynx',
    maintainer_email='valiantlynxz@gmail.com',
    description='the package for serial motor demo',
    license='Look at the LICENSE file in the root directory',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gui = serial_motor_demo.gui:main',
            'driver = serial_motor_demo.driver:main'
        ],
    },
)
