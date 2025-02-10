# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Holistic Fusion'
copyright = '2025, Julian Nubert, Turcan Tuna, Jonas Frey, Cesar Cadena, Katherine J. Kuchenbecker, Shehryar Khattak, Marco Hutter'
author = 'Julian Nubert, Turcan Tuna, Jonas Frey, Cesar Cadena, Katherine J. Kuchenbecker, Shehryar Khattak, Marco Hutter'
release = 'January 2025'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Add myst_parser to the extensions list
extensions = [
    'myst_parser', # Markdown support
    'sphinx_rtd_theme', # Read the Docs theme
    'sphinx.ext.githubpages', # GitHub Pages, create .nojekyll file
]

# Specify the file extensions to look for
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'  # 'alabaster'
html_static_path = ['_static']

# Add theme-specific options
html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 6,
    'includehidden': True,
    'titles_only': False
}
