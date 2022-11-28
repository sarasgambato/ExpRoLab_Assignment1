# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath('..'))
subprocess.call('doxygen Doxyfile.in', shell=True)
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Assignment_1'
copyright = '2022, Sara Sgambato'
author = 'Sara Sgambato'
release = 'v.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    "sphinx.ext.napoleon",
    'sphinx.ext.inheritance_diagram',
    'breathe'
]

highlight_language = 'python'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
napoleon_custom_sections = [('Returns', 'params_style')]

language = 'English'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'alabaster'
html_static_path = ['_static']

# -- Extension configuration -------------------------------------------------

# -- Options for intersphinx extension ---------------------------------------
# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'https://docs.python.org/': None}

# -- Options for todo extension ----------------------------------------------
# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True

# -- Options for breathe
breathe_projects = {
"turtlesim_controller": "_build/xml/"
}
breathe_default_project = "turtlesim_controller"
breathe_default_members = ('members', 'undoc-members')
