from setuptools import find_packages, setup

package_name = 'action_challenge_py'

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
    maintainer='exiew',
    maintainer_email='josephtandio27@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_one_axis_server = action_challenge_py.move_one_axis_server:main',
            'move_one_axis_client = action_challenge_py.move_one_axis_client:main',
        ],
    },
)
