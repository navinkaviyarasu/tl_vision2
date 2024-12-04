from setuptools import setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nk',
    maintainer_email='navin_k@zohomail.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_bridge = vision.vio_bridge:main',
            'vio_bridge_px4 = vision.vio_bridge_px4:main',
            'vicon_bridge = vision.vicon_bridge:main',
            'vio_bridge_beta = vision.vio_px4_bridge_beta:main'
        ],
    },
)
