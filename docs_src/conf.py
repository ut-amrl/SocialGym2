# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../'))


# -- Project information -----------------------------------------------------

project = 'Social Gym 2.0'
copyright = '2022, Zayne Sprague, Jarrett Holtz, Rohan Chandra, Joydeep Biswas'
author = 'Zayne Sprague, Jarrett Holtz, Rohan Chandra, Joydeep Biswas'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.napoleon', 'myst_parser']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_favicon = 'favicon.ico'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
html_logo = "https://drive.google.com/uc?id=1F1hEwQiFuwT7OGwYrJ6t8O9NqrlNk4RH"
html_theme_options = {
    'logo_only': False,
    'display_version': False,
    'logo': {
        "text": "My awesome documentation",
    }
}

autodoc_mock_imports = ['roslib', 'amrl_msgs', 'rospy', 'roslaunch', 'gym', 'numpy', 'tensorboardX', 'pettingzoo',
                        'ut_multirobot_sim', 'graph_navigation', 'visualization_msgs', 'geometry_msgs', 'torch',
                        'optuna', 'stable_baselines3', 'sensor_msgs', 'cv2', 'supersuit', 'sb3_contrib', 'cv_bridge']

