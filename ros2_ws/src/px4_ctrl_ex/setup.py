from setuptools import find_packages, setup

package_name = 'px4_ctrl_ex'

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
    maintainer='uav',
    maintainer_email='uav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_motion_ctrl = px4_ctrl_ex.px4_motion_ctrl:main',
            'keyboard_gz_ctrl = px4_ctrl_ex.keyboard_gz_ctrl:main'
        ],
    },
)
