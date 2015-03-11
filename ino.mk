#!/usr/bin/make -f

# Default INO_DIR is the current location of the "Production" clone.
# It can be overriden on the command line like this:
# INO_DIR=/home/jdoe/ino   ./ino.mk  clean

INO_DIR ?= /p/clanton/users/pmicolet/ino


INO_LIB_DIR:= ${INO_DIR}/ino_lib_dependencies

# "official" and documented trick to escape spaces
empty :=
space := ${empty} ${empty}

# Include all subdirectories with have at least one .py file
# FIXME: this library list should rather somehow come from the ino repo
_INO_LIBS :=  ${INO_DIR}  $(sort $(dir $(wildcard ${INO_LIB_DIR}/*/*.py)))
# Change spaces to colons
INO_LIBS :=  $(subst ${space},:,${_INO_LIBS})


.PHONY: compile clean


compile: sketch_image
	cd $^ && PYTHONPATH=${INO_LIBS} ${INO_DIR}/bin/ino build -d ${PWD} -m izmir_fd -c ../ino_conf.txt -v


sketch_image:
	PYTHONPATH=${INO_LIBS} ${INO_DIR}/setup_sketch.py  ${INO_DIR}/bin/ino


clean:
	${RM} -r sketch_image
	PYTHONPATH=${INO_LIBS} ${INO_DIR}/setup_sketch.py ${INO_DIR}/bin/ino --clean
