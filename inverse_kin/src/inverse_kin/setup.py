from setuptools import find_packages, setup

package_name = 'inverse_kin'

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
		'server = inverse_kin.server:main',
		'auto_arm_controller = inverse_kin.auto_arm_controller:main',
		'expectedmotor = inverse_kin.expected_motor:main',
		'encoder1 = inverse_kin.encoder1:main'
	    ],
	},

)
