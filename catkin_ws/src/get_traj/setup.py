from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# These are the ONLY parameters to put in the setup.py file, don't change anything else.
setup_args = generate_distutils_setup(
    packages=['get_traj'], # name of the package
    package_dir={'': 'nodes'}, # directory of .py files
    requires=['geometry_msgs', 'rospy'] # dependencies, they have to be the same as "CMakeLists.txt" and "package.xml" and written as Python strings, for example: ["rospy", "std_msgs"].
)

setup(**setup_args)
