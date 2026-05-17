import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot3_llm_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('turtlebot3_llm_nav2/script/*.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Jeonghee Seo',
    author_email='sjh0621@gachon.ac.kr',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Nav2 + LLM integration package'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = turtlebot3_llm_nav2.script.waypoint_navigator:main'
        ],
    },
)
