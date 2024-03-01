# Building documentation
This README explains how to build the documentation for the project. It uses `sphinx` to generate the documentation from the source code.

## Installation
To install the required dependencies, run the following command:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Building the documentation
To build the documentation, run the following command:

```bash
sphinx-build -M html source build
```
This will generate the documentation in the `build` directory. It can be viewed by opening the `build/html/index.html` file in a web browser.
