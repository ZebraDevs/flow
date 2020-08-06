# Contributing to Flow

## Pull Requests


### Style checks

`clang-format` is used automatically format C++ code in this library. Style checks are required to pass in order for a pull request to be merged.


#### Local setup

This repository contains a `pre-commit` configuration which can be used run code formatting on local changes. To make use of this with local branches, first ensure that `clang-format` and `pre-commit` are installed:

```
$ sudo apt install clang-format python-pip
$ pip install pre-commit
```

#### Running code formatting manually

To auto-format changes in a local branch manually, run:

```
$ pre-commit run -a
```

This will reformat any code that does not conform with the clang [style settings](.clang-format).


#### Running code formatting automatically on commits

To run style checks on each new commit, run the following to setup a commit hook:

```
$ pre-commit install

pre-commit installed at .git/hooks/pre-commit
```
