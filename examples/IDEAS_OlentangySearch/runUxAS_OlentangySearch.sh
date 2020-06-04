#! /bin/bash

SAVE_DIR=$(pwd)

RM_DATAWORK="rm -R ./datawork"
RM_LOG="rm -R ./log"

BIN="../../../build_debug/uxas"

mkdir -p RUNDIR_OlentangySearch
cd RUNDIR_OlentangySearch
$RM_DATAWORK
$RM_LOG
$BIN -cfgPath ../cfg_OlentangySearch.xml

