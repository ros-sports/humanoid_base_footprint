from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['humanoid_base_footprint'],
    #scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)