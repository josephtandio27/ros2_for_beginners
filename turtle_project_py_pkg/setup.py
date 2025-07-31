from setuptools import find_packages, setup

package_name = 'turtle_project_py_pkg'

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
    maintainer_email='exiew@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_manager = turtle_project_py_pkg.turtle_manager:main',
            'turtle_controller = turtle_project_py_pkg.turtle_controller:main'
        ],
    },
)
