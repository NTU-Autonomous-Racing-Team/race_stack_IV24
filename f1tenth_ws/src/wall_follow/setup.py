from setuptools import setup

package_name = 'wall_follow'

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
    maintainer='Luthov',
    maintainer_email='lukettl15@gmail.com',
    description='Wall Following Node for F1Tenth',
    license='GNU General Public License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow = wall_follow.wall_follow:main'
        ],
    },
)
