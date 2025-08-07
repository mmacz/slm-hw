#!/bin/bash

VENV_NAME=.venv

if [ -d $VENV_NAME ]; then rm -rf $VENV_NAME; fi

uv venv $VENV_NAME
source $VENV_NAME/bin/activate
uv pip install -r requirements.txt
deactivate

echo "Venv created under: $VENV_NAME"

