from setuptools import find_packages, setup

package_name = 'object_identifier'

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
    maintainer='lukas',
    maintainer_email='KneisslLukas@web.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_identifier = object_identifier.object_identifier:main',
            'cylinder_identifier = object_identifier.cylinder_identifier:main'
        ],
    },
)
