# Added to allows to find the source files:
import os
import sys
# Path working using local sphinx compilation
#sys.path.append(os.path.abspath('C:/Python/PhoxModbus/phox_modbus/'))

# Path workin in all conditions (used for compilation on RTD servers)
sys.path.insert(0, os.path.abspath('../../'))

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'phox_modbus'
copyright = '2023, PHOXENE'
author = 'Aurélien PLANTIN'
# Import de la version depuis le fichier projet
# Bidouille APLAN
import sys
sys.path.append("..")
from modbus import __version__
release = __version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Add any Sphinx extension module names here, as strings. They can be extensions
# coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = ['sphinx.ext.autodoc']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix of source filenames.
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# List of directories, relative to source directory, that shouldn't be searched
# for source files.
exclude_trees = ['_build']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# The theme to use for HTML and HTML Help pages.  Major themes that come with
# Sphinx are currently 'default' and 'sphinxdoc'.
html_theme = 'sphinx_rtd_theme'

# The name of an image file (relative to this directory) to place at the top
# of the sidebar.
html_logo = 'https://www.phoxene.com/wp-content/uploads/2018/09/flash-strobe-manufacturer-phoxene.png'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

