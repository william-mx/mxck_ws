from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['utils'],  # Ensure 'utils' is listed here
    package_dir={'': 'src'}  # Map to the correct directory
)

setup(**d)