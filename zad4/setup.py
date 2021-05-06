from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'zad4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('zad4/*.py')),
        (os.path.join('share', package_name), glob('zad4/*.txt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateusz',
    maintainer_email='mateuszzembron@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'no_kdl = zad4.nokdl_dkin:main',
                'kdl = zad4.kdl_dkin:main',
                'interpolation_srv = zad4.jint_srv:main',
                'interpolation_client = zad4.jint_client:main'
        ],
    },
)
