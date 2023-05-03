TEX := main.tex
BIB := main.bib
CONTENT := $(wildcard content/*.tex)
PDF := main.pdf

.PHONY: all
all: $(PDF)

$(PDF): $(TEX) $(BIB) $(CONTENT)
	latexmk $<

.PHONY: clean
clean:
	latexmk -c

.PHONY: distclean
distclean: clean
	latexmk -CA
	rm -f main.pdf
