# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-micro-controller'
# noinspection PyShadowingBuiltins
copyright = '2024, Ivan Kondratyev (Inkaros) & Sun Lab'
authors = ['Ivan Kondratyev (Inkaros)', 'Jasmine Si']
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
    'sphinx_rtd_theme',    # To format the documentation HTML using ReadTheDocs format.
    'sphinx_rtd_dark_mode' # Enables dark mode for RTD theme.
]

templates_path = ['_templates']
exclude_patterns = []

# Breathe configuration
breathe_projects = {"ataraxis-micro-controller": "./doxygen/xml"}
breathe_default_project = "ataraxis-micro-controller"
breathe_doxygen_config_options = {
    'ENABLE_PREPROCESSING': 'YES',
    'MACRO_EXPANSION': 'YES',
    'EXPAND_ONLY_PREDEF': 'NO',
    'PREDEFINED': 'PACKED_STRUCT='
}

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
