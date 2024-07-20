from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Models
        ('share/' + package_name + '/models', ['models/Simple1.ttm']),
        # Launchs
        ('share/' + package_name + '/launch', ['launch/cop_sim_spawn_robot.launch.py'])
    ],
    install_requires=['setuptools'],
    requires=['coppeliasim_zmqremoteapi_client'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='joao.t06@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cop_sim_spawn_robot = simulation.cop_sim_spawn_robot:main'
        ],
    },
)
