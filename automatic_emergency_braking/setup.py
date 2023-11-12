from setuptools import setup

package_name = 'automatic_emergency_braking'

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
    maintainer='bentjh01',
    maintainer_email='benjamintehjh01@gmail.com',
    description='Automatic Emergency Braking node using time to collision',
    license='GNU General Public License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'AEB = automatic_emergency_braking.AEB:main'
        ],
    },
)
