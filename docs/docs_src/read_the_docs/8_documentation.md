# Building Documentation

Documentation is automatically built and deployed to GitHub Pages on every push to `main` via the
[docs GitHub Action](https://github.com/leggedrobotics/holistic_fusion/actions/workflows/docs.yml).
On pull requests, the documentation is built (but not deployed) to catch errors early.

The sections below describe how to build the documentation locally.

## Read The Docs

This documentation uses ReadTheDocs and Sphinx.

### Building

The documentation dependencies are managed via [uv](https://docs.astral.sh/uv/). To build:

```bash
cd docs/docs_src/read_the_docs
uv run make html
```

This automatically creates a local `.venv`, installs the required packages (`sphinx`, `myst-parser`, `sphinx-rtd-theme`), and runs the Sphinx build.

To update the served documentation locally, copy the build output:

```bash
cd <repo_root>
cp -r docs/docs_src/read_the_docs/_build/html/* docs/docs/
```

## Doxygen

We also use Doxygen to generate documentation for the C++ code. You can install it via:

```bash
sudo apt install doxygen graphviz
```

Then, you can generate the documentation by running:
```bash
doxygen docs/docs_src/doxy/Doxyfile
```


