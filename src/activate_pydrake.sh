#!/bin/sh
export PYTHONPATH=~/git/manipulation:${PYTHONPATH}
conda activate diva
jupyter notebook
