from setuptools import find_packages, setup

package_name = 'ros2_yolo'

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
    maintainer='amsl',
    maintainer_email='amsl@todo.todo',
    description='The ros2_yolo package',
    license='todo',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'predictor = ros2_yolo.predict:main',
        ],
    },
)
