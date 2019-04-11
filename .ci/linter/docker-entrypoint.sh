#!/bin/bash
set -e

pylint --rcfile linter/.pylintrc gym_gazebo2

res=$?

if [ $res -ne 0 ]; then
	echo "Linter error"
	exit 123
fi
exit 0
