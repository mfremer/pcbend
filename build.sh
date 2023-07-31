#!/bin/bash

THREADS=${1:-1}

# Pipeline
cmake --build pipeline/build -j "$THREADS"

# Unfolder
cmake --build pipeline/build/unfolder -j "$THREADS"

# Chamferer
cmake --build pipeline/build/chamferer -j "$THREADS"
