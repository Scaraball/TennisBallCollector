from setuptools import setup

package_name = 'scaraball_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/launch.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mourtaza',
    maintainer_email='mourtaza.kassamaly@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'go_to_point = scaraball_control.go_to_point:main',
	'control_pinces = scaraball_control.control_pince:main'
        ],
    },
)
