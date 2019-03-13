# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = LSD-SLAM
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

## --- Top-level targets ---

all: uml html

clean:
	rm -rf doxyoutput/ api/
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

## --- UML diagrams ---

UML_DIAGRAMS= uml/slamsystem.uml
UML_SVGS := $(patsubst uml/%,_static/uml/%,$(UML_DIAGRAMS:.uml=.svg))

## Blank pre-requisite otherwise Sphinx become a pre-req of everything...
uml/%.uml:

_static/uml/%.svg: uml/%.uml
	cat $< | docker run --rm -i think/plantuml > $@

uml: $(UML_SVGS)

.PHONY: help Makefile uml all clean

## ---  ----

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
