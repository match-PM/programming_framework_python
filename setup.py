from setuptools import setup

package_name = 'programming_fw_pkg'

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
    maintainer='pmlab',
    maintainer_email='koch.leon1998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movegroup_cam1_call_service = programming_fw_pkg.movegroup_cam1_call_service:main',
            'movegroup_laser_call_service = programming_fw_pkg.movegroup_laser_call_service:main',
            'movegroup_tool_call_service = programming_fw_pkg.movegroup_tool_call_service:main',
            'gui_test = programming_fw_pkg.python_gui:main',
            'new_python_gui = programming_fw_pkg.new_python_gui:main',
        ],
    },
)
