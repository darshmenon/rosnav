from setuptools import setup

package_name = 'rosnav_check'

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
    description='One report from all rosnav doctors: merge findings, correlate root causes, CI exit code.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['rosnav_check = rosnav_check.cli:main']},
)
