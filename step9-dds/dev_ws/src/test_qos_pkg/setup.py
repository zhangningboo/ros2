from setuptools import find_packages, setup

package_name = 'test_qos_pkg'

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
    maintainer='ubuntu',
    maintainer_email='zhangningbo21@mails.ucas.ac.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld_pub   = test_qos_pkg.helloworld_pub:main',
            'helloworld_sub   = test_qos_pkg.helloworld_sub:main',
        ],
    },
)
