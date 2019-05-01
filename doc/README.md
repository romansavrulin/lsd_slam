This directory contains documentation for LSD-SLAM.
It is written in
[Sphinx](http://www.sphinx-doc.org/en/master/) and designed to be hosted at
[Read The Docs](https://lsd-slam.readthedocs.io/en/latest/).

In addition to the "standard" Sphinx documentation, we also use
[Breathe](https://github.com/michaeljones/breathe) and
[Exhale](https://github.com/svenevs/exhale) to pull in Doxygen-generated in-line
code docs.

Everything is coordinated through the Makefile in this directory.  In addition
to the standard Sphinx targets, the Makefile also includes:

  - `make uml` uses a Docker-ized version of PlantUML to translate the UML
  descriptions in `uml/` to diagrams in `_static/uml/`

  - `make all` combines `make uml` and Sphinx's `make html`

Since Breathe/Exhale are Sphinx plug-ins, the code documentation is
automatically regenerated as part of the Sphinx build.
