#! /bin/bash

INST_DIR=/usr/local/eSEAT
MKDIR="/bin/mkdir -p "
CP="/usr/bin/sudo /bin/cp "
#FILES="seatml.xsd rtc.conf"
FILES="rtc.conf"
DIRS="html libs 3rd_party examples bin ros_samples"

echo "Install Python2"
sudo apt-get install python-pip
sudo apt-get install python-tk
sudo apt-get install python-yaml
sudo apt-get install python-lxml

echo "Install eSEAT"
sudo python setup.py install

[ -d $INST_DIR ] || $MKDIR $INST_DIR

for f in $FILES; do
  $CP $f $INST_DIR
done

for d in $DIRS; do
  $CP -r $d $INST_DIR
done
