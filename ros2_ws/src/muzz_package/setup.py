from setuptools import setup

package_name = 'muzz_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[muzz_package],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzz',
    maintainer_email='muzzammil.rehman@ontariotechu.net',
    description='TODO:Beginner client libraries tutorials practice package',
    license='TODO:Apache Liscence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = muzz_package.my_node:main'
        ],
    },
)
