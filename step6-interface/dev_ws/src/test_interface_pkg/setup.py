from setuptools import find_packages, setup

package_name = 'test_interface_pkg'

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
            'pub_custom_msg   = test_interface_pkg.pub_custom_msg:main',
            'sub_custom_msg   = test_interface_pkg.sub_custom_msg:main',
            'service_server   = test_interface_pkg.service_server:main',
            'service_client   = test_interface_pkg.service_client:main',
        ],
    },
)
