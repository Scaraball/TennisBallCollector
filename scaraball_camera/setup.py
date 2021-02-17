from setuptools import setup

package_name = 'scaraball_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
('share/' + package_name + '/launch', ['launch/position.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paul',
    maintainer_email='paul.pineau@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position = scaraball_camera.position:main'
        ],
    },
)
