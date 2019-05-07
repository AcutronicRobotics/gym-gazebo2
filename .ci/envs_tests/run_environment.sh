#!/bin/bash

set -e

source /root/gym-gazebo2/provision/mara_setup.sh

cd /root/gym-gazebo2/tests/

python3 test_mara_envs.py
