from setuptools import find_packages, setup

package_name = 'haru_2026_pkg'

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
    maintainer='altair',
    maintainer_email='robotic.engineer.dream@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ps3_node = haru_2026_pkg.ps3_node:main',
            'roboware_node = haru_2026_pkg.roboware_node:main',
            'can_node = haru_2026_pkg.can_node:main',
            'button_cannode = haru_2026_pkg.button_cannode:main',
            'ps4_robo_node = haru_2026_pkg.ps4_robo_node:main',
            'ps4_node = haru_2026_pkg.ps4_node:main'
        ],
    },
)
