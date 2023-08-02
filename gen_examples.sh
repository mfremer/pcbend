#!/bin/bash
set -e

THREADS=${1:-4}

./generate.sh batman-092 small-dihedral $1 ""
./generate.sh cat-102 small-dihedral $1 "24"
./generate.sh dome-081 small-dihedral $1 ""
./generate.sh icosa-020 small-dihedral $1 ""
./generate.sh sqtorus-048 small-perimeter $1 "8"
./generate.sh star-024 small-perimeter $1 "14"
./generate.sh architecture-080 small-perimeter $1 "14"

./generate.sh batman-092 big-dihedral $1 ""
./generate.sh cat-102 big-dihedral $1 "24"
./generate.sh dome-081 big-dihedral $1 ""
./generate.sh icosa-020 big-dihedral $1 ""
./generate.sh sqtorus-048 big-perimeter $1 "8"
./generate.sh star-024 big-perimeter $1 "14"
./generate.sh architecture-080 big-perimeter $1 ""
