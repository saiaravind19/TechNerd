# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'TechNerd'
copyright = ''
author = 'Sai Aravind'

release = '0.1'
version = '0.1.0'

html_logo = 'docs/source/images/logo.png'
# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.graphviz',
    'sphinx.ext.ifconfig',
    'sphinx.ext.mathjax',
    'sphinx_copybutton',
    'sphinx_multiversion',
    'sphinx_tabs.tabs',
    'sphinx_book_theme',
    'sphinxcontrib.mermaid',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}

intersphinx_disabled_domains = ['std']
source_suffix = ['.rst','.md']
master_doc = 'index'

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_book_theme'
html_theme_options = {
    "logo_only": True,
    "repository_url": "https://github.com/saiaravind/TechNerd",
    "use_repository_button": True,
    "use_issues_button": True,
    "use_download_button": True,
    "use_fullscreen_button": True,
}

# -- Options for EPUB output
epub_show_urls = 'footnote'
# MathJax configuration
mathjax_path = 'https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-AMS-MML_HTMLorMML'
