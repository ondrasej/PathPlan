.PHONY: all clean run

GHC_OPTS=-odir bin -hidir bin -XFlexibleInstances -XMultiParamTypeClasses -XFunctionalDependencies -XRankNTypes
GHC_COMPILE_OPTS=-c

all: bin/pathplan

run:
	ghci $(GHC_OPTS) -isrc PathPlan

bin/pathplan: src/PathPlan.hs bin
	ghc $(GHC_OPTS) -o bin/pathplan -main-is PathPlan.main --make PathPlan -isrc
	
bin:
	mkdir -p bin

clean:
	rm bin/*
