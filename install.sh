#! /bin/bash

INST_DIR=/usr/local/eSEAT
MKDIR="/bin/mkdir -p "
CP="/bin/cp "
FILES="seatml.xsd rtc.conf"
DIRS="html libs 3rd_party examples bin"

[ -d $INST_DIR ] || $MKDIR $INST_DIR

for f in $FILES; do
  $CP $f $INST_DIR
done

for d in $DIRS; do
  $CP -r $d $INST_DIR
done
