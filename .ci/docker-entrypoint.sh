#!/bin/sh
set -e
cd /gym-gazebo2
pylint --rcfile linter/.pylintrc gym-gazebo2

res=$?

if [ $res -ne 0 ]; then
	echo "Linter error"
	exit 123
fi
exit 0
