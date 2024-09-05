from setuptools import setup

package_name = 'skiros2_pyrobosim_lib'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/main.launch.py']),
        ('share/' + package_name + "/owl", ['owl/robi_robot_description.owl', 'owl/p1_scene.turtle']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
)
