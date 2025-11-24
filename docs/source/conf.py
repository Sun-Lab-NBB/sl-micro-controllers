# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'sl-micro-controllers'
# noinspection PyShadowingBuiltins
copyright = '2025, Sun (NeuroAI) lab'
authors = ['Ivan Kondratyev']
release = '3.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
    'sphinx_rtd_theme',    # To format the documentation HTML using ReadTheDocs format.
    'sphinx_rtd_dark_mode' # Enables dark mode for RTD theme.
]

templates_path = ['_templates']
exclude_patterns = []

# Breathe configuration
breathe_projects = {"sl-micro-controllers": "./doxygen/xml"}
breathe_default_project = "sl-micro-controllers"
breathe_doxygen_config_options = {
    'ENABLE_PREPROCESSING': 'YES',
    'MACRO_EXPANSION': 'YES',
    'EXPAND_ONLY_PREDEF': 'NO',
}

# Disables the dark mode by default.
default_dark_mode = False

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
