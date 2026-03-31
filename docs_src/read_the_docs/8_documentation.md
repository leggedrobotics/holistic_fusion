# Building Documentation

## Read The Docs

This documentation uses ReadTheDocs and Sphinx.

### Building

The documentation dependencies are managed via [uv](https://docs.astral.sh/uv/). To build:

```bash
cd docs_src/read_the_docs
uv run make html
```

This automatically creates a local `.venv`, installs the required packages (`sphinx`, `myst-parser`, `sphinx-rtd-theme`), and runs the Sphinx build.

After this, currently we are manually copying the content of the _build folder to the `docs` folder in the root of the
repository, which is used by the Github pages.

```bash
cd <repo_root>
cp -r docs_src/read_the_docs/_build/html/* docs/docs/
```

## Doxygen

We also use Doxygen to generate documentation for the C++ code. You can install it via:

```bash
sudo apt install doxygen graphviz
```

Then, you can generate the documentation by running:
```bash
doxygen docs/doxy/Doxyfile
```


