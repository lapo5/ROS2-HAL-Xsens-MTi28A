from setuptools import setup

package_name = 'hal_xsens_mti_28a'

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
    maintainer='marco_lapo',
    maintainer_email='marco.lapolla5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "imu_node = hal_xsens_mti_28a.imu_node:main",
        ],
    },
)
