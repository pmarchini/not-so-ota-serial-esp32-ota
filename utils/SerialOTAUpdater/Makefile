.PHONY: install lint test format

install:
	pip3 install -r ./requirements.txt

lint:
	pylint *.py

test:
	pytest

format:
	autopep8 --in-place --aggressive --aggressive *.py

all: lint test format