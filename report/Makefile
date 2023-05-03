LATEXMK := latexmk

TEX := main.tex
BIB := main.bib
CONTENT := $(wildcard content/*.tex)

PDF := main.pdf

################################################################################

.PHONY: all
all: $(PDF)

$(PDF): $(TEX) $(BIB) $(CONTENT)
	$(LATEXMK) $<

.PHONY: clean
clean:
	$(LATEXMK) -c

.PHONY: distclean
distclean: clean
	$(LATEXMK) -CA
	rm -f $(PDF)
