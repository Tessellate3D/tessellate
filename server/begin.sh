#!/bin/bash
# set -e

# virtualenv -p python3 venv || python3 -m venv venv || python -m venv venv
# source venv/bin/activate
pip install --upgrade pip
pip install --requirement requirements.txt
pip install numpy
pip install pyserial


FLASK_APP=server.py flask run
