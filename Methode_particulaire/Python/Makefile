SCRIPT_FOL  = script
DATA_FOL    = data
INCLUDE_FOL = include

PROG        = mission.py
CLASSE      = particule.py
PROG_CPP    = display_from_cpp.py


DATA = Robot_States.txt

default: run

run:
	echo "\n>>>> Run launched ..." & PYTHONPATH="${INCLUDE_FOL}" python3 "${SCRIPT_FOL}/${PROG}" 

cpp:
	python3 "${SCRIPT_FOL}/${PROG_CPP}" "${DATA_FOL}/${DATA}"

test:
	python3 "${INCLUDE_FOL}/${CLASSE}"


