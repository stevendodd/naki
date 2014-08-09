#!/bin/sh

export BOARD=gpio328
make -f ../../sketchbook/Makefile && ../../sketchbook/upload.sh payload.hex
