from setuptools import setup

package_name = 'moveit2_data_collector'

setup(
 name=package_name,
 version='0.0.0',
 packages=[
     'moveit2_data_collector',
     'moveit2_data_collector.robot_workspaces',
     ],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Peter David Fagan',
 maintainer_email='peterdavidfagan@gmail.com',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     "console_scripts": [
        ],
    },
)

