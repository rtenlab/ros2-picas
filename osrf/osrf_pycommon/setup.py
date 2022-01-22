from setuptools import find_packages
from setuptools import setup


install_requires = [
    'setuptools',
]
package_excludes = ['tests*', 'docs*']
packages = find_packages(exclude=package_excludes)

package_name = 'osrf_pycommon'

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=install_requires,
    python_requires='>=3.5',
    zip_safe=True,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='http://osrf-pycommon.readthedocs.org/',
    keywords=['osrf', 'utilities'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description="Commonly needed Python modules, "
                "used by Python software developed at OSRF",
    license='Apache 2.0',
    test_suite='tests',
)
