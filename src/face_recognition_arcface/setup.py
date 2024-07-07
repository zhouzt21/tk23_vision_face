from setuptools import find_packages, setup

package_name = 'face_recognition_arcface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ( 'lib/' , ['lib/libarcsoft_face_engine.so', 'lib/libarcsoft_face.so' ] ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzt',
    maintainer_email='zhou-zt21@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receptionist_face_task = face_recognition_arcface.receptionist_face_task:main',
        ],
    },
)
