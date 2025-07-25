from setuptools import find_packages, setup
from glob import glob

package_name = 'launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/'+ package_name+'/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nk',
    maintainer_email='navin_k@zohomail.in',
    description='Common launch files for various nodes for VIO based navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
