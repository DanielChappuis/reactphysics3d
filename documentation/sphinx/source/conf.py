# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# read version from ../VERSION
version = open('../../../VERSION').readlines()[0]
# The full version, including alpha/beta/rc tags.

# The master toctree document.
master_doc = 'index'

project = 'ReactPhysics3D'
copyright = '2024, Daniel Chappuis'
author = 'Daniel Chappuis'
release = "v" + version

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["breathe", "exhale"]

exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'

# Breathe Configuration
breathe_default_project = "ReactPhysics3D"

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "doxygenStripFromPath":  "../../../",
    # Heavily encouraged optional argument (see docs)
    "rootFileTitle":         "C++ API Reference",
    "afterTitleDescription":         "This section describes the API of the library.",
    # Suggested optional arguments
    "createTreeView":        True,
    "exhaleExecutesDoxygen": False,
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

highlight_language = 'c++'
