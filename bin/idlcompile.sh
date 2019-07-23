#!/bin/bash

if [ y"$1" = "y" ]; then
  echo "Usage $0 <idl_file>"
else
  if [ ! -e rtm_idl ]; then
    /bin/mkdir rtm_idl
  fi
  if [ -d rtm_idl ]; then
    /usr/bin/omniidl -bpython -Crtm_idl $1
  else
    echo "rtm_idl is not directory, please remove rtm_idl"
  fi
fi
