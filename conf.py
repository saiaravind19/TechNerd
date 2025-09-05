# Configuration file for the Sphinx documentation builder.

project = 'Sai Aravind'
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
    'sphinxcontrib.mermaid',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}

intersphinx_disabled_domains = ['std']
source_suffix = ['.rst', '.md']
master_doc = 'index'

templates_path = ['_templates']

# -- Options for HTML output (PyData Theme)

html_theme = 'pydata_sphinx_theme'
html_theme_options = {
    "logo": {
        "text": "Sai Aravind",
        "image_light": "docs/source/images/logo.png",
        "image_dark": "docs/source/images/logo.png",
    },
    "navbar_align": "content",
    "navbar_center": [],  # Empty to remove project tabs
    "navbar_end": ["navbar-icon-links"],
    "navbar_persistent": ["search-button"],
    "primary_sidebar_end": ["sidebar-ethical-ads"],
    "secondary_sidebar_items": ["page-toc"],
    "footer_start": ["copyright"],
    "footer_end": ["theme-version"],
    "show_prev_next": False,
    "search_bar_text": "Search the docs...",
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/saiaravind19",
            "icon": "fab fa-github-square",
            "type": "fontawesome",
        },
        {
            "name": "LinkedIn",
            "url": "https://www.linkedin.com/in/saiaravindexplorer/",
            "icon": "fab fa-linkedin",
            "type": "fontawesome",
        },
        {
            "name": "Email",
            "url": "mailto:bplacearavind@gmail.com",
            "icon": "fas fa-envelope",
            "type": "fontawesome",
        },
        {
            "name": "Resume",
            "url": "https://drive.google.com/file/d/1Xlu0VwFYl18RiJV-ikHpRSfR4niD70vo/view?usp=sharing",
            "icon": "fas fa-file-pdf",
            "type": "fontawesome",
        },
    ],
    "announcement": "🚀 Hello Folks! Welcome to my robotics portfolio and tech blog!",
}

# Build settings
html_title = 'Sai Aravind - Robotics & Tech Blog'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_js_files = ['disable-tooltips.js']
html_copy_source = False
html_show_sourcelink = True

# -- Options for EPUB output
epub_show_urls = 'footnote'

# MathJax configuration
mathjax_path = 'https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-AMS-MML_HTMLorMML'

# Exclude patterns
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'blog/**', 'venv/**']
