from setuptools import find_packages, setup
import os
import glob

package_name = 'buff_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'msg'), glob.glob('msg/*.msg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jrh',
    maintainer_email='jrh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = buff_tracker.self_aiming:main'
        ],
    },
)

