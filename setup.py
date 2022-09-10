from setuptools import setup

package_name = 'ros_remocon'

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
    maintainer='DaiGuard',
    maintainer_email='zenith.or.w@gmail.com',
    description='remote controller for ros package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remocon_ps5 = ros_remocon.remocon_ps5:main'
        ],
    },
)
