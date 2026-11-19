from setuptools import setup

package_name = 'lio_doctor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darshmenon',
    maintainer_email='darshmenon02@gmail.com',
    description='Is my lidar(-inertial) odometry healthy? Live checks plus a KISS-ICP / RKO-LIO / LIO-SAM readiness preflight.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['lio_doctor = lio_doctor.doctor:main']},
)
