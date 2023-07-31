#!/bin/sh

THREADS=${1:-1}

# Pipeline
cmake --build pipeline/build -j "$THREADS" 
