# Configuration file for the Sphinx documentation builder.

import subprocess

subprocess.call('python3 -m pip install virtualenvwrapper --break-system-packages', shell=True)
subprocess.call('cd ../../; ./build_multiverse.sh --only-src', shell=True)

print('****************************************')
print('conf.py')
print('****************************************')

# -- Project information

project = 'MultiverseFramework'
copyright = 'Institute for Artificial Intelligence, University of Bremen'

release = '0.1'
version = '0.0.1'

# -- Doxygen and breath
subprocess.call('mkdir -p build/html/; cd ..; doxygen', shell=True)
breathe_projects = { "MultiverseFramework": "build/html/doxygen_generated/xml" }
breathe_default_project = "MultiverseFramework"

# -- General configuration
extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.imgmath', 
    'sphinx.ext.todo', 
    'sphinx.ext.graphviz',
    'sphinxcontrib.video',
    'breathe', 
    'myst_parser'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['build']

#highlight_language = 'c++' # default: 'python3'

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
# html_extra_path = ['build/html', '../html']

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# -- Options for EPUB output
epub_show_urls = 'footnote'

def setup(app):
    app.add_css_file('css/style.css')
