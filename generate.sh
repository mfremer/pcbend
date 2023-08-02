#!/bin/bash
#set -e # stop on error

# General variables
DIR_MESH="./data/meshes"
DIR_CFG="./data/configs"

EXT_MESH=".off"
EXT_CFG=".cfg"
EXT_SHEET=".sheet"
EXT_FACES=".fis"
EXT_CHAMFER="_chamfer.off"
EXT_LED=".led"
EXT_MAP=".map"

AWK_COUNT_FPNR="./data/count_failed_pnr.awk"
AWK_COUNT_FVER="./data/count_failed_verif.awk"
AWK_STATS="./data/test.awk"

# Documentation
if [ "$#" -eq 0 ]; then
  echo "Usage: $0 MESH CONFIG #PARALLEL \"CONN_SFIDS\"" >&2
  echo -e "AVAILABLE MESHES:\t" $(ls "$DIR_MESH"/*"$EXT_MESH" | xargs basename -a -s "$EXT_MESH")
  echo -e "AVAILABLE CONFIGS:\t" $(ls "$DIR_CFG"/*"$EXT_CFG" | xargs basename -a -s "$EXT_CFG")
  exit 1
fi
if [[ "$#" -lt 2 || "$#" -gt 4 ]]; then
  echo "Wrong number of arguments, please run without any arguments for help."
  exit 1
fi

# Input args
PROJ_NAME=$1
CONF_NAME=${2:-"default"}
BATCH_SIZE=${3:-1}
CONNSFIDS=${4:-""}
SPLIT=1
if [ "$BATCH_SIZE" -eq 1 ]; then
  SPLIT=0
fi

# Data variables
DIR_PROJ="./data/results/""$PROJ_NAME""_""$CONF_NAME"
DIR_FACES="$DIR_PROJ/faces"
DIR_SVGS="$DIR_PROJ/svgs"

FILE_MESH="$DIR_MESH/$PROJ_NAME$EXT_MESH"
FILE_CFG="$DIR_CFG/$CONF_NAME$EXT_CFG"
FILE_SHEET="$DIR_PROJ/$PROJ_NAME$EXT_SHEET"
FILE_FACES="$DIR_FACES/$PROJ_NAME$EXT_FACES"
FILE_CHAMFER="$DIR_PROJ/$PROJ_NAME$EXT_CHAMFER"
FILE_LED="$DIR_PROJ/$PROJ_NAME$EXT_LED"
FILE_MAP="$DIR_PROJ/$PROJ_NAME$EXT_MAP"

LOG_FILE="$DIR_PROJ/log.txt"

function print_info(){
  echo "PIPELINE"
  echo -e "Mesh\t\t""$FILE_MESH"
  echo -e "Config\t\t""$FILE_CFG"
  echo -e "Connector(s)\t""$CONNSFIDS"
  echo -e "#Parallel\t""$BATCH_SIZE"
  echo -e "Output in\t""$DIR_PROJ"
}

function validate_info(){
  read -p "Continue? (Y/N): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1
}

function unfolder_task(){
  echo -e "BEGIN\tunfolder_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" >> $LOG_FILE
  ./pipeline/build/unfolder/Unfolder "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET"
  echo -e "END\tunfolder_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" >> $LOG_FILE
}

function chamferer_task(){
  echo -e "BEGIN\tchamferer_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_CHAMFER" >> $LOG_FILE
  ./pipeline/build/chamferer/Chamferer "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_CHAMFER"
  echo -e "BEGIN\tchamferer_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_CHAMFER" >> $LOG_FILE
}

function cutter_task(){
  echo -e "BEGIN\tcutter_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$CONNSFIDS" "$SPLIT" >> $LOG_FILE
  ./pipeline/build/cutter/Cutter "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$CONNSFIDS" "$SPLIT";
  echo -e "END\tcutter_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$CONNSFIDS" "$SPLIT" >> $LOG_FILE
}

function placer_task(){
  echo -e "BEGIN\tplacer_task\t" "$1" >> $LOG_FILE
  ./pipeline/build/placer/Placer "$FILE_CFG" "$1";
  echo -e "END\tplacer_task\t" "$1" >> $LOG_FILE
}

function router_task(){
  echo -e "BEGIN\trouter_task\t" "$1" >> $LOG_FILE
  ./pipeline/build/router/Router "$FILE_CFG" "$1";
  echo -e "END\trouter_task\t" "$1" >> $LOG_FILE
}

function verifier_task(){
  echo -e "BEGIN\tverifier_task\t" "$1" >> $LOG_FILE
  ./pipeline/build/verifier/Verifier "$FILE_CFG" "$1";
  echo -e "END\tverifier_task\t" "$1" >> $LOG_FILE
}

function layout_task(){
  echo -e "BEGIN\tlayout_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$DIR_SVGS" >> $LOG_FILE
  ./pipeline/build/layouter/Layout2SVG "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$DIR_SVGS"
  echo -e "END\tlayout_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$DIR_SVGS" >> $LOG_FILE
}

function orderer_task(){
  echo -e "BEGIN\torderer_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$FILE_MAP" >> $LOG_FILE
  ./pipeline/build/orderer/Orderer "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$FILE_MAP"
  echo -e "END\torderer_task\t" "$FILE_CFG" "$FILE_SHEET" "$FILE_FACES" "$FILE_MAP" >> $LOG_FILE
}

function ledifier_task(){
  echo -e "BEGIN\tledifier_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_FACES" "$FILE_LED" >> $LOG_FILE
  ./pipeline/build/placer/Ledifier "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_FACES" "$FILE_LED"
  echo -e "END\tledifier_task\t" "$FILE_CFG" "$FILE_MESH" "$FILE_SHEET" "$FILE_FACES" "$FILE_LED" >> $LOG_FILE
}

function count_failed_pnr(){
  echo $(gawk -f ${AWK_COUNT_FPNR} "$1")
}

function count_failed_pnr_all(){
  NUM_FPNR=0
  for file in "$DIR_FACES"/*; do
    VAL=$(count_failed_pnr "$file")
    NUM_FPNR=$((NUM_FPNR + VAL))
#    echo "'$file' -> '$VAL' (TOT '$NUM_FPNR')"
  done
  echo $NUM_FPNR
}

function count_failed_ver(){
  echo $(gawk -f ${AWK_COUNT_FVER} "$1")
}

function count_failed_ver_all(){
  NUM_FVER=0
  for file in "$DIR_FACES"/*; do
    VAL=$(count_failed_ver "$file")
    NUM_FVER=$((NUM_FVER + VAL))
#    echo "'$file' -> '$VAL' (TOT '$NUM_FVER')"
  done
  echo $NUM_FVER
}

function pnr_loop(){
  echo -e "BEGIN\tpnr_loop()\t" "$1" >> $LOG_FILE
  while # do while loop
    placer_task "$1"
    router_task "$1"
    [ $(count_failed_pnr "$1") -ne 0 ]
  do true; done
  echo -e "END\tpnr_loop()\t" "$1" >> $LOG_FILE
}

function main_loop(){
  echo -e "BEGIN\tmain_loop()\t" "$1" >> $LOG_FILE
  while
    pnr_loop "$1"
    verifier_task "$1"
    [ $(count_failed_ver "$1") -ne 0 ]
  do true; done
  echo -e "END\tmain_loop()\t" "$1" >> $LOG_FILE
}

function concat_faces(){
  if [ "$SPLIT" -eq 1 ]; then
    if [ -f "$FILE_FACES" ]; then
      rm "$FILE_FACES"
    fi
    cat "$DIR_FACES"/* > "$FILE_FACES"
  fi
}

# https://unix.stackexchange.com/questions/103920/parallelize-a-bash-for-loop
function main_par(){
  N=1 # default of 1, only batch if split into multiple files
  if [ "$SPLIT" -eq 1 ]; then
    N=$((BATCH_SIZE))
  fi
  
  for file in "$DIR_FACES"/*; do
  # filter faces that need PNR to avoid opening a ton of windows
    (
#      echo "Processing file '$file'"
      main_loop "$file"
    ) &
    
    # allow to execute up to $N jobs in parallel
    if [[ $(jobs -r -p | wc -l) -ge $N ]]; then
      # now there are $N jobs already running, so wait here for any job
      # to be finished so there is a place to start next one.
      wait -n
    fi
  done
  
  wait
}

function pipeline(){
  print_info
  validate_info
  mkdir -p "$DIR_PROJ" "$DIR_FACES" "$DIR_SVGS"
  > $LOG_FILE
  TIME_START=$SECONDS
  unfolder_task
  chamferer_task
  cutter_task
  main_par
  concat_faces
  layout_task
  orderer_task
  ledifier_task
  TIME_DURATION=$(($SECONDS - TIME_START))
  gawk -f $AWK_STATS $FILE_FACES
  echo -e "Total runtime:\t" "$TIME_DURATION" "s"
  echo -e "Total runtime:\t" "$TIME_DURATION" "s" >> $LOG_FILE
  rm -f *.res
}

# SEPARATE PIPELINE
#> $LOG_FILE
#cutter_task
#main_par
#concat_faces
#layout_task

# MAIN
pipeline
