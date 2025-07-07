# CoHAN 2.0 Documentation

This repository contains the documentation for the CoHAN 2.0 (Cooperative Human-Aware Navigation) framework.

## Online Documentation

The documentation is available online at: [https://LAAS-HRI.github.io/CoHAN2.0_docs/](https://LAAS-HRI.github.io/CoHAN2.0_docs/)

## Building the Documentation Locally

To build the documentation locally, follow these steps:

```bash
# Navigate to the docs directory
cd docs

# Install dependencies (if not already installed)
pip install sphinx sphinx_rtd_theme breathe

# Build the HTML documentation
make html

# Open the documentation in your browser
xdg-open build/html/index.html  # On Linux
# open build/html/index.html    # On macOS
# start build/html/index.html   # On Windows
```

## Documentation Structure

- `docs/source/`: Contains the source RST files for the documentation
- `docs/build/html/`: Contains the built HTML documentation
- `docs/docs/`: Contains Doxygen output (XML and HTML)
