#!/bin/bash

if [ y"$1" = "y" ]; then
  echo "Usage $0 <idl_file>"
else
  if [ ! -e rtm_l ]; then
    /bin/mkdir rtm_l
  fi
  if [ -d rtm_l ]; then
    /usr/bin/omniidl -bpython -Crtm_l $1
  else
    echo "rtm is not directory, please remove rtm"
  fi
fi
