# Minimal makefile for Sphinx documentation
#

ifeq ($(VERBOSE),1)
  Q =
else
  Q = @
endif

# You can set these variables from the command line.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
PYTHON        ?= python3
SPHINXPROJ    = "ROS2 Planning System Documentation"
SOURCEDIR     = .
BUILDDIR      = _build

DOC_TAG      ?= development
RELEASE      ?= latest
PUBLISHDIR    = /tmp/plansys2

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@echo ""
	@echo "make site"
	@echo "   build every version of versions.json in _site/"
	@echo "make publish"
	@echo "   build the site and publish it to the gh-pages branch of PlanSys2.github.io"

.PHONY: help Makefile site publish

# Generate the doxygen xml (for Sphinx) and copy the doxygen html to the
# api folder for publishing along with the Sphinx-generated API docs.

html:
	$(Q)$(SPHINXBUILD) -t $(DOC_TAG) -b html -d $(BUILDDIR)/doctrees $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS) $(O)

# Remove generated content (Sphinx and doxygen)

clean:
	rm -fr $(BUILDDIR)

# Copy material over to the GitHub pages staging repo
# along with a README

# Every version of versions.json (latest from this tree, the others from their tags) in _site/
site:
	$(Q)$(PYTHON) scripts/build_site.py

publish: site
	rm -fr $(PUBLISHDIR)
	git clone --reference . https://github.com/PlanSys2/PlanSys2.github.io.git $(PUBLISHDIR)
	cd $(PUBLISHDIR) && \
	git checkout gh-pages && \
	git config user.email "fmrico@gmail.com" && \
	git config user.name "fmrico"
	rm -fr $(PUBLISHDIR)/*
	cp -r _site/* $(PUBLISHDIR)
	cp scripts/.nojekyll $(PUBLISHDIR)/.nojekyll
	# cp scripts/CNAME $(PUBLISHDIR)/CNAME
	cd $(PUBLISHDIR) && \
	git add -A && \
	git diff-index --quiet HEAD || \
	(git commit -s -m "[skip ci] publish $(RELEASE)" && git push origin)
