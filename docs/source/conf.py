# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# Add the source folder to the path to be able to automatically generate the API documentation
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join('..', '..', 'src')))

project = "Grasping Pipeline"
copyright = "2024, Alexander Haberl"
author = "Alexander Haberl"
release = "0.1"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


extensions = [
    "sphinx_copybutton",      # adds a copy button to code blocks
    "sphinx.ext.autodoc",     # include docstrings from modules
    "sphinx.ext.autosummary", # create summaries of modules, aka API docs
    "sphinx.ext.napoleon",    # support for Google and numpy-style docstrings
    "sphinx_rtd_theme"
]

copybutton_exclude = ".linenos, .gp"

templates_path = ["_templates"]
exclude_patterns = []

# Mock packages that are not available when building the documentation, aka all ROS packages
# This is the case because autodoc imports the modules instead of just parsing the docstrings, because python is a dynamic language 
autodoc_mock_imports = ['hsrb_interface', 'rospy', 'tf', 'smach', 'actionlib', 'smach_ros', 'sensor_msgs', 'std_srvs', 'geometry_msgs', 
                        'table_plane_extractor', 'move_base_msgs', 'vision_msgs', 'v4r_util', 'grasping_pipeline_msgs', 'moveit_commander', 
                        'moveit_msgs', 'trajectory_msgs', 'numpy', 'handover', 'yaml', 'open3d', 'ros_numpy', 'visualization_msgs', 
                    'message_filters', 'std_msgs', 'robot_llm', 'object_detector_msgs', 'PyKDL', 'tmc_geometric_shapes_msgs',
                    'tf_conversions', 'tmc_placement_area_detector', 'robokudo_msgs']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = "furo"
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
