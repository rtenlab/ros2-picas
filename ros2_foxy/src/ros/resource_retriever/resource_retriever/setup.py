from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['resource_retriever']
d['scripts'] = []
d['package_dir'] = {'': 'src'}

setup(**d)
