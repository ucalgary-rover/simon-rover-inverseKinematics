from setuptools import find_packages, setup

package_name = 'inverseKin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='magdy',
    maintainer_email='Magdy.hafez9123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
		'server = inverseKin.server:main',
		'auto_arm_controller = inverseKin.auto_arm_controller:main',
		'expectedmotor = inverseKin.expected_motor:main',
		'encoder1 = inverseKin.encoder1:main'
	    ],
	},

)
