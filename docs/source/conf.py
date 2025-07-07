# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'CoHAN2.0'
copyright = '2025, Phani Teja Singamaneni'
author = 'Phani Teja Singamaneni'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []

# -- GitHub Pages Settings --------------------------------------------------
# Ensure the master_doc is set properly for GitHub Pages
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'alabaster'
html_theme = 'sphinx_rtd_theme' 
html_static_path = ['_static']
def setup(app):
    app.add_css_file("custom.css")
    
# html_sidebars = {
#     '**': ['globaltoc.html', 'relations.html', 'searchbox.html']
# }

# For GitHub Pages
html_baseurl = 'https://laas-hri.github.io/CoHAN2.0_docs/'

import os
import sys
# from exhale.utils import makeCustomSpecificationsMapping
sys.path.insert(0, os.path.abspath('.'))

extensions = ['breathe']

# Path to Doxygen XML
breathe_projects = {
    "CoHAN2.0": "../docs/xml"
}
breathe_default_project = "CoHAN2.0"

# def customSpec(typeName):
#     mapping = {
#         "class": [":members:", ":private-members:", ":protected-members:", ":undoc-members:", ":static-members:"],
#         "struct": [":members:", ":private-members:", ":protected-members:", ":undoc-members:", ":static-members:"],
#     }
#     return mapping.get(typeName, [])

# exhale_args = {
#     "containmentFolder": "./api",
#     "rootFileName": "library_root.rst",
#     "rootFileTitle": "API Reference",
#     "doxygenStripFromPath": "/home/phani/ros_ws/CoHAN2.0/src/",
#     "createTreeView": True,
# # "customSpecificationsMapping": makeCustomSpecificationsMapping(customSpec),
#     }

