from setuptools import setup

package_name = 'rqt_top'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dan Lazewatsky',
    maintainer='Dirk Thomas, Dan Lazewatsky',
    maintainer_email='dthomas@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'RQT plugin for monitoring ROS processes.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_top = ' + package_name + '.main:main',
        ],
    },
)
