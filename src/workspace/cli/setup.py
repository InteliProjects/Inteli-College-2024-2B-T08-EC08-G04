from setuptools import find_packages, setup

package_name = 'cli'

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
    maintainer='J.A.R.B.A.S.',
    maintainer_email='mario.medeiros@sou.inteli.edu.br',
    description='CLI controller for jarbinhas',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = cli.cli:app',
        ],
    },
)
