# Building Documentation

## Read The Docs

This documentation uses ReadTheDocs and Sphinx.

### Dependencies

To build the documentation locally, you need to have Sphinx installed. You can install it via pip:

```bash
pip install sphinx
```

Moreover, as we use markdown files, you need to install the `myst_parser` package:

```bash
pip install myst-parser
```

Moreover, to make the documentation look nice, we use the `sphinx_rtd_theme`:

```bash
pip install sphinx_rtd_theme
```

Then, you can build the documentation by running:

```bash
cd docs_src/read_the_docs
make html
```

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


