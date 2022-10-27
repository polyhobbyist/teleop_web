import os
from setuptools import setup
from glob import glob

package_name = 'teleop_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lou Amadio',
    author_email='lou@polyhobbyist.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Teleop Web Server.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'teleop_web = teleop_web.teleop_web:main',
        ],
    },
  data_files=[
        # ...
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'webroot'), glob('webroot/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),      
    ]    
)