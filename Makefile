.PHONY: docs docs-clean docs-open

docs:
	doxygen Doxyfile

docs-clean:
	rm -rf docs/html docs/xml docs/doxygen-warnings.log

docs-open:
	open docs/html/index.html
