## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pr2_rviz_ik'],
    package_dir={'': 'src'},
    requires=['rospy', 'actionlib', 'control_msgs', 'kinematics_msgs', 'numpy', 'tf', 'geometry_msgs', 'visualization_msgs', 'interactive_', 'std_msgs']
)


setup(**setup_args)
